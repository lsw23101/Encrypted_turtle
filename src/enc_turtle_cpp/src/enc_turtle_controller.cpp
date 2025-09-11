#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "enc_turtle_cpp/msg/encrypted_data.hpp"
#include "openfhe/pke/openfhe.h"
#include <thread>
#include <cmath>

// Cereal 등록 매크로 추가
CEREAL_CLASS_VERSION(lbcrypto::CryptoParametersBGVRNS, 1)
CEREAL_CLASS_VERSION(lbcrypto::DCRTPoly, 1)
CEREAL_CLASS_VERSION(lbcrypto::SchemeBGVRNS, 1)
CEREAL_CLASS_VERSION(lbcrypto::CiphertextImpl<lbcrypto::DCRTPoly>, 1)

// 타입 등록
CEREAL_REGISTER_TYPE(lbcrypto::CryptoParametersBGVRNS)
CEREAL_REGISTER_TYPE(lbcrypto::SchemeBGVRNS)
CEREAL_REGISTER_TYPE(lbcrypto::CiphertextImpl<lbcrypto::DCRTPoly>)

// 다형성 관계 등록
CEREAL_REGISTER_POLYMORPHIC_RELATION(lbcrypto::CryptoParametersBase<lbcrypto::DCRTPoly>, lbcrypto::CryptoParametersBGVRNS)
CEREAL_REGISTER_POLYMORPHIC_RELATION(lbcrypto::SchemeBase<lbcrypto::DCRTPoly>, lbcrypto::SchemeBGVRNS)

using namespace lbcrypto;

class EncTurtleController : public rclcpp::Node {
public:
  EncTurtleController() : Node("enc_turtle_controller") {
    // 암호화 설정
    CCParams<CryptoContextBGVRNS> parameters;
    parameters.SetPlaintextModulus(65537);
    parameters.SetMultiplicativeDepth(1);  // 곱셈이 필요 없으므로 1로 설정
    parameters.SetSecurityLevel(SecurityLevel::HEStd_128_classic);
    parameters.SetRingDim(8192);  // 덧셈만 하므로 더 작은 값으로 설정

    cc = GenCryptoContext(parameters);
    cc->Enable(PKE);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    kp = cc->KeyGen();

    if (kp.secretKey) {
        cc->EvalSumKeyGen(kp.secretKey);  // 덧셈 키만 생성
    } else {
        RCLCPP_ERROR(this->get_logger(), "Key generation failed!");
    }

    // QoS 설정
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();
    
    // 암호화 관련 구독 및 발행
    encrypted_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_pose", qos,
        std::bind(&EncTurtleController::enc_pose_callback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_result", qos);

    // 기존 터틀 컨트롤러 기능
    turtle1_state_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/state", 10,
      std::bind(&EncTurtleController::target_pose_callback, this, std::placeholders::_1));
      
    turtle2_state_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/state", 10,
      std::bind(&EncTurtleController::current_pose_callback, this, std::placeholders::_1));
      
    control_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle2/control_command", 10);
      
    // 초기 위치 설정
    target_x_ = 5.0;
    target_y_ = 5.0;
  }

private:
  // 암호화 관련 콜백
  void enc_pose_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      RCLCPP_INFO(this->get_logger(), "Received data size: %zu bytes", msg->data.size());
      std::string data_str(msg->data.begin(), msg->data.end());
      std::stringstream ss(data_str);
      
      if (msg->data_type == 0) {  // X coordinate
        RCLCPP_INFO(this->get_logger(), "Processing X coordinate");
        Serial::Deserialize(ciphertext_x, ss, SerType::BINARY);
        RCLCPP_INFO(this->get_logger(), "X coordinate deserialized successfully");
      } else if (msg->data_type == 1) {  // Y coordinate
        RCLCPP_INFO(this->get_logger(), "Processing Y coordinate");
        Serial::Deserialize(ciphertext_y, ss, SerType::BINARY);
        RCLCPP_INFO(this->get_logger(), "Y coordinate deserialized successfully");
        
        // Y 좌표를 받았으면 계산 수행
        auto ct_result = cc->EvalAdd(ciphertext_x, ciphertext_y);
        RCLCPP_INFO(this->get_logger(), "Addition performed successfully");
        
        std::stringstream ss_result;
        Serial::Serialize(ct_result, ss_result, SerType::BINARY);
        std::string result_str = ss_result.str();
        std::vector<uint8_t> result_data(result_str.begin(), result_str.end());
        RCLCPP_INFO(this->get_logger(), "Result serialized, size: %zu bytes", result_data.size());
        
        auto msg_out = enc_turtle_cpp::msg::EncryptedData();
        msg_out.data = result_data;
        msg_out.data_type = 2;  // Result type
        
        result_pub_->publish(msg_out);
        RCLCPP_INFO(this->get_logger(), "Result published successfully");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in encryption: %s", e.what());
    }
  }

  // 기존 터틀 컨트롤러 콜백
  void target_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    target_x_ = msg->x;
    target_y_ = msg->y;
    RCLCPP_DEBUG(this->get_logger(), 
      "Updated target position: x=%.2f, y=%.2f", msg->x, msg->y);
  }

  void current_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    // 위치 오차 계산
    double err_x = target_x_ - msg->x;
    double err_y = target_y_ - msg->y;
    double err_dist = std::sqrt(err_x * err_x + err_y * err_y);
    
    // 목표 방향 계산
    double desired_theta = std::atan2(err_y, err_x);
    double err_theta = desired_theta - msg->theta;
    
    // 각도 정규화 (-π ~ π)
    while (err_theta > M_PI) {
      err_theta -= 2.0 * M_PI;
    }
    while (err_theta < -M_PI) {
      err_theta += 2.0 * M_PI;
    }
    
    // 제어 게인
    const double Kp_dist = 0.9;
    const double Kp_theta = 2.0;
    
    // 제어 명령 계산
    geometry_msgs::msg::Twist cmd;
    if (err_dist < 0.1) {  // 더 작은 임계값으로 수정
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = Kp_dist * err_dist;
      cmd.angular.z = Kp_theta * err_theta;
    }
    
    // 제어 명령 발행
    control_command_pub_->publish(cmd);
    RCLCPP_DEBUG(this->get_logger(), 
      "Published control command: linear=%.2f, angular=%.2f", 
      cmd.linear.x, cmd.angular.z);
  }

  // 암호화 관련 변수
  CryptoContext<DCRTPoly> cc;
  KeyPair<DCRTPoly> kp;
  Ciphertext<DCRTPoly> ciphertext_x;
  Ciphertext<DCRTPoly> ciphertext_y;

  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr encrypted_sub_;
  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr result_pub_;

  // 기존 터틀 컨트롤러 변수
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_state_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_command_pub_;
  double target_x_;
  double target_y_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncTurtleController>());
  rclcpp::shutdown();
  return 0;
}
