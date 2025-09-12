#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "enc_turtle_cpp/msg/encrypted_data.hpp"
#include "openfhe/pke/openfhe.h"
#include <thread>
#include <cmath>
#include <sstream>

// Cereal 등록 매크로
CEREAL_CLASS_VERSION(lbcrypto::CryptoParametersBGVRNS, 1)
CEREAL_CLASS_VERSION(lbcrypto::DCRTPoly, 1)
CEREAL_CLASS_VERSION(lbcrypto::SchemeBGVRNS, 1)
CEREAL_CLASS_VERSION(lbcrypto::CiphertextImpl<lbcrypto::DCRTPoly>, 1)
CEREAL_REGISTER_TYPE(lbcrypto::CryptoParametersBGVRNS)
CEREAL_REGISTER_TYPE(lbcrypto::SchemeBGVRNS)
CEREAL_REGISTER_TYPE(lbcrypto::CiphertextImpl<lbcrypto::DCRTPoly>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(lbcrypto::CryptoParametersBase<lbcrypto::DCRTPoly>, lbcrypto::CryptoParametersBGVRNS)
CEREAL_REGISTER_POLYMORPHIC_RELATION(lbcrypto::SchemeBase<lbcrypto::DCRTPoly>, lbcrypto::SchemeBGVRNS)
// EvalMult(재선형) 키 직렬화용 등록
CEREAL_CLASS_VERSION(lbcrypto::EvalKeyRelinImpl<lbcrypto::DCRTPoly>, 1)
CEREAL_REGISTER_TYPE(lbcrypto::EvalKeyRelinImpl<lbcrypto::DCRTPoly>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(lbcrypto::Serializable, lbcrypto::EvalKeyRelinImpl<lbcrypto::DCRTPoly>)

using namespace lbcrypto;

class EncTurtleController : public rclcpp::Node {
public:
  EncTurtleController() : Node("enc_turtle_controller") {
    // QoS
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    // === Setup 수신: 이제 EvalMultKey만 받음 ===
    sub_emk_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
      "fhe_evalmult", qos,
      std::bind(&EncTurtleController::on_evalmult, this, std::placeholders::_1));

    // === 런타임 채널 ===
    encrypted_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
      "encrypted_pose", qos,
      std::bind(&EncTurtleController::enc_pose_callback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>("encrypted_result", qos);

    // 기존 터틀 컨트롤러 기능(그대로 유지)
    turtle1_state_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/state", 10,
      std::bind(&EncTurtleController::target_pose_callback, this, std::placeholders::_1));

    turtle2_state_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/state", 10,
      std::bind(&EncTurtleController::current_pose_callback, this, std::placeholders::_1));

    control_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle2/control_command", 10);
  }

private:
  // --- EvalMultKey 수신 (cc는 암호문에서 획득하므로 도착 전까지 버퍼링만) ---
  void on_evalmult(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg){
    std::string s(msg->data.begin(), msg->data.end());
    if (!run_cc_) {
      pending_emk_ = std::move(s);
      RCLCPP_INFO(this->get_logger(), "[Controller] EvalMultKey buffered (wait for ciphertext context)");
      return;
    }
    std::stringstream ss(pending_emk_.empty() ? s : pending_emk_);
    run_cc_->DeserializeEvalMultKey(ss, SerType::BINARY);
    got_emk_ = true;
    emk_registered_ = true;
    pending_emk_.clear();
    RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context");
  }

  // --- 암호문 수신/연산 ---
  void enc_pose_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      std::string data_str(msg->data.begin(), msg->data.end());
      std::stringstream ss(data_str);

      if (msg->data_type == 0) { // X
        Serial::Deserialize(ciphertext_x_, ss, SerType::BINARY);
        got_x_ = true;

        // 첫 암호문에서 컨텍스트 확보
        if (!run_cc_) {
          run_cc_ = ciphertext_x_->GetCryptoContext();
          run_cc_->Enable(KEYSWITCH);
          run_cc_->Enable(LEVELEDSHE);
          // 버퍼된 evalmult 키 있으면 지금 등록
          if (!pending_emk_.empty() && !emk_registered_) {
            std::stringstream ess(pending_emk_);
            run_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
            emk_registered_ = true;
            got_emk_ = true;
            pending_emk_.clear();
            RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context (from buffer)");
          }
        }
        RCLCPP_INFO(this->get_logger(), "Encrypted X received");

      } else if (msg->data_type == 1) { // Y
        Serial::Deserialize(ciphertext_y_, ss, SerType::BINARY);
        got_y_ = true;

        if (!run_cc_) {
          run_cc_ = ciphertext_y_->GetCryptoContext();
          run_cc_->Enable(KEYSWITCH);
          run_cc_->Enable(LEVELEDSHE);
          if (!pending_emk_.empty() && !emk_registered_) {
            std::stringstream ess(pending_emk_);
            run_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
            emk_registered_ = true;
            got_emk_ = true;
            pending_emk_.clear();
            RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context (from buffer)");
          }
        }
        RCLCPP_INFO(this->get_logger(), "Encrypted Y received");
      }

      // 두 암호문 다 있으면 연산
      if (got_x_ && got_y_) {
        // 안전 체크: 같은 컨텍스트인지
        if (ciphertext_x_->GetCryptoContext() != ciphertext_y_->GetCryptoContext()) {
          RCLCPP_ERROR(this->get_logger(), "X and Y contexts mismatch");
          return;
        }

        // 평가키가 아직이면 등록 시도
        if (!emk_registered_ && !pending_emk_.empty()) {
          std::stringstream ess(pending_emk_);
          run_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
          emk_registered_ = true;
          got_emk_ = true;
          pending_emk_.clear();
          RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context (late)");
        }

        if (!got_emk_) {
          RCLCPP_WARN(this->get_logger(), "[Controller] EvalMult key not registered yet; skipping mult");
        }

        // === 같은 컨텍스트(run_cc_)로 연산 ===
        auto ct_sum  = run_cc_->EvalAdd(ciphertext_x_, ciphertext_y_);
        enc_turtle_cpp::msg::EncryptedData out_sum;
        {
          std::stringstream ss_out;
          Serial::Serialize(ct_sum, ss_out, SerType::BINARY);
          auto s = ss_out.str();
          out_sum.data.assign(s.begin(), s.end());
          out_sum.data_type = 2; // x+y
          result_pub_->publish(out_sum);
        }

        if (got_emk_) {
          auto ct_prod = run_cc_->EvalMult(ciphertext_x_, ciphertext_y_);
          enc_turtle_cpp::msg::EncryptedData out_prod;
          std::stringstream ss_out2;
          Serial::Serialize(ct_prod, ss_out2, SerType::BINARY);
          auto s2 = ss_out2.str();
          out_prod.data.assign(s2.begin(), s2.end());
          out_prod.data_type = 3; // x*y
          result_pub_->publish(out_prod);
        }

        RCLCPP_INFO(this->get_logger(), "Published enc (x+y)%s",
          got_emk_ ? " and (x*y)" : " (mult skipped: no eval key)");
        // 필요하면 다음 프레임을 위해 리셋
        got_x_ = got_y_ = false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in homomorphic ops: %s", e.what());
    }
  }

  // --- 기존 터틀 컨트롤러 콜백 ---
  void target_pose_callback(const turtlesim::msg::Pose::SharedPtr msg){
    target_x_ = msg->x;
    target_y_ = msg->y;
  }

  void current_pose_callback(const turtlesim::msg::Pose::SharedPtr msg){
    double err_x = target_x_ - msg->x;
    double err_y = target_y_ - msg->y;
    double err_dist = std::sqrt(err_x * err_x + err_y * err_y);
    double desired_theta = std::atan2(err_y, err_x);
    double err_theta = std::clamp(desired_theta - msg->theta, -M_PI, M_PI);

    const double Kp_dist = 0.9;
    const double Kp_theta = 2.0;

    geometry_msgs::msg::Twist cmd;
    if (err_dist < 0.1) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = Kp_dist * err_dist;
      cmd.angular.z = Kp_theta * err_theta;
    }
    control_command_pub_->publish(cmd);
  }

  // 상태
  CryptoContext<DCRTPoly> run_cc_;             // 실제 연산 컨텍스트(암호문에서 획득)
  bool got_emk_ = false;
  bool emk_registered_ = false;
  std::string pending_emk_;                    // run_cc_ 준비 전 도착한 evalmult 키 버퍼

  Ciphertext<DCRTPoly> ciphertext_x_, ciphertext_y_;
  bool got_x_ = false, got_y_ = false;

  // ROS2
  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr sub_emk_;
  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr encrypted_sub_;
  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr result_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_state_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_command_pub_;
  double target_x_ = 5.0, target_y_ = 5.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncTurtleController>());
  rclcpp::shutdown();
  return 0;
}
