#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "enc_turtle_cpp/msg/encrypted_data.hpp"
#include "openfhe/pke/openfhe.h"
#include <thread>
#include <chrono>
#include <iomanip>
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
// EvalMult 키 직렬화용 등록
CEREAL_CLASS_VERSION(lbcrypto::EvalKeyRelinImpl<lbcrypto::DCRTPoly>, 1)
CEREAL_REGISTER_TYPE(lbcrypto::EvalKeyRelinImpl<lbcrypto::DCRTPoly>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(lbcrypto::Serializable, lbcrypto::EvalKeyRelinImpl<lbcrypto::DCRTPoly>)

using namespace lbcrypto;

class EncTurtlePlant : public rclcpp::Node {
public:
  EncTurtlePlant() : Node("enc_turtle_plant") {
    // ===== 스케일 상수 =====
    SCALE_XY_   = 10;                   // x,y 스케일
    SCALE_SUM_  = SCALE_XY_;            // 합 결과는 ÷10
    SCALE_PROD_ = SCALE_XY_ * SCALE_XY_; // 곱 결과는 ÷100

    // ===== 암호화 설정 =====
    CCParams<CryptoContextBGVRNS> parameters;
    parameters.SetPlaintextModulus(65537);
    parameters.SetMultiplicativeDepth(1);       // x*y 1회
    parameters.SetSecurityLevel(SecurityLevel::HEStd_NotSet);
    parameters.SetRingDim(8192);

    cc = GenCryptoContext(parameters);
    cc->Enable(PKE);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);
    cc->Enable(KEYSWITCH);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    kp = cc->KeyGen();
    if (kp.secretKey) {
      cc->EvalSumKeyGen(kp.secretKey);     // (선택)
      cc->EvalMultKeyGen(kp.secretKey);    // ★ 곱셈용 평가키 생성
    } else {
      RCLCPP_ERROR(this->get_logger(), "Key generation failed!");
    }

    // ===== QoS =====
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    // ===== Setup 채널: EvalMultKey만 송신 =====
    emk_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>("fhe_evalmult", qos);

    // ===== 런타임 채널 =====
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", qos,
        std::bind(&EncTurtlePlant::pose_callback, this, std::placeholders::_1));

    result_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_result", qos,
        std::bind(&EncTurtlePlant::result_callback, this, std::placeholders::_1));

    encrypted_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_pose", qos);

    // ===== 기존 터틀 플랜트 기능 유지 =====
    turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&EncTurtlePlant::turtle1_pose_callback, this, std::placeholders::_1));

    turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10,
      std::bind(&EncTurtlePlant::turtle2_pose_callback, this, std::placeholders::_1));

    turtle1_state_pub_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/state", 10);
    turtle2_state_pub_ = this->create_publisher<turtlesim::msg::Pose>("/turtle2/state", 10);
    turtle2_cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

    control_command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle2/control_command", 10,
      std::bind(&EncTurtlePlant::control_command_callback, this, std::placeholders::_1));

    // 초기 셋업(★ EvalMultKey만 1회 송신)
    setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                    [this]{ send_evalmult_once(); });
  }

private:
  // 초기: EvalMultKey만 송신
  void send_evalmult_once(){
    if (emk_sent_) return;
    emk_sent_ = true;

    std::stringstream ss_emk;
    cc->SerializeEvalMultKey(ss_emk, SerType::BINARY);
    std::string semk = ss_emk.str();

    enc_turtle_cpp::msg::EncryptedData memk;
    memk.data.assign(semk.begin(), semk.end());
    memk.data_type = 101; // evalmult 표시용
    emk_pub_->publish(memk);

    RCLCPP_INFO(this->get_logger(), "[Plant] Setup sent: EvalMultKey only");
  }

  // 암호화 관련 콜백 (x,y 스케일 적용해서 암호화·전송)
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    try {
      original_x_ = msg->x;
      original_y_ = msg->y;

      // 전체 루프 시작
      loop_start_time_ = std::chrono::high_resolution_clock::now();

      // 암호화 시작
      encryption_start_ = std::chrono::high_resolution_clock::now();
      std::vector<int64_t> x_coord = {static_cast<int64_t>(msg->x * SCALE_XY_)};
      std::vector<int64_t> y_coord = {static_cast<int64_t>(msg->y * SCALE_XY_)};

      auto plaintext_x = cc->MakePackedPlaintext(x_coord);
      auto plaintext_y = cc->MakePackedPlaintext(y_coord);

      auto ciphertext_x = cc->Encrypt(kp.publicKey, plaintext_x);
      auto ciphertext_y = cc->Encrypt(kp.publicKey, plaintext_y);

      auto encryption_end = std::chrono::high_resolution_clock::now();
      auto encryption_time = std::chrono::duration_cast<std::chrono::microseconds>(
        encryption_end - encryption_start_).count() / 1000.0;

      // 직렬화
      serialization_start_ = std::chrono::high_resolution_clock::now();
      std::stringstream ss_x, ss_y;
      Serial::Serialize(ciphertext_x, ss_x, SerType::BINARY);
      Serial::Serialize(ciphertext_y, ss_y, SerType::BINARY);

      std::string x_str = ss_x.str();
      std::string y_str = ss_y.str();
      std::vector<uint8_t> x_data(x_str.begin(), x_str.end());
      std::vector<uint8_t> y_data(y_str.begin(), y_str.end());

      auto serialization_end = std::chrono::high_resolution_clock::now();
      auto serialization_time = std::chrono::duration_cast<std::chrono::microseconds>(
        serialization_end - serialization_start_).count() / 1000.0;

      std::stringstream info;
      info << "\n========== 새로운 암호화 통신 시작 ==========\n"
           << "원본 데이터: x=" << std::fixed << std::setprecision(3) << original_x_
           << ", y=" << original_y_ << " (합: " << original_x_ + original_y_ << ")\n"
           << "암호화 시간: " << encryption_time << " ms\n"
           << "직렬화 시간: " << serialization_time << " ms\n"
           << "데이터 크기(x): " << x_data.size() << " bytes\n"
           << "----------------------------------------";
      RCLCPP_INFO(this->get_logger(), "%s", info.str().c_str());

      // X 좌표 전송
      auto msg_x = enc_turtle_cpp::msg::EncryptedData();
      msg_x.data = x_data;
      msg_x.data_type = 0;
      encrypted_pub_->publish(msg_x);

      // Y 좌표 전송
      auto msg_y = enc_turtle_cpp::msg::EncryptedData();
      msg_y.data = y_data;
      msg_y.data_type = 1;
      encrypted_pub_->publish(msg_y);

      // 터틀봇 제어를 위한 상태 정보 전송
      turtle1_state_pub_->publish(*msg);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in encryption: %s", e.what());
    }
  }

  // 결과 수신(덧셈/곱셈 모두 처리: 합은 ÷SCALE_XY, 곱은 ÷(SCALE_XY^2))
  void result_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      auto deserialization_start = std::chrono::high_resolution_clock::now();
      std::string data_str(msg->data.begin(), msg->data.end());
      std::stringstream ss(data_str);

      Ciphertext<DCRTPoly> ciphertext;
      Serial::Deserialize(ciphertext, ss, SerType::BINARY);

      auto decryption_start = std::chrono::high_resolution_clock::now();
      auto deserialization_time = std::chrono::duration_cast<std::chrono::microseconds>(
        decryption_start - deserialization_start).count() / 1000.0;

      Plaintext plaintext;
      cc->Decrypt(kp.secretKey, ciphertext, &plaintext);
      plaintext->SetLength(1);

      int64_t raw = static_cast<int64_t>(plaintext->GetPackedValue()[0]);
      double value = 0.0;
      std::string what = "(unknown)";

      if (msg->data_type == 2) {         // sum(x+y)
        value = static_cast<double>(raw) / static_cast<double>(SCALE_SUM_);
        what = "(x+y)";
      } else if (msg->data_type == 3) {  // prod(x*y)
        value = static_cast<double>(raw) / static_cast<double>(SCALE_PROD_);
        what = "(x*y)";
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      auto decryption_time = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - decryption_start).count() / 1000.0;
      auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - loop_start_time_).count() / 1000.0;

      std::stringstream info;
      info << "\n========== 암호화 통신 결과 ==========\n"
           << "복호화된 결과 " << what << ": " << std::fixed << std::setprecision(3) << value << "\n"
           << "역직렬화 시간: " << deserialization_time << " ms\n"
           << "복호화 시간: " << decryption_time << " ms\n"
           << "전체 루프 시간: " << total_time << " ms\n"
           << "======================================\n";
      RCLCPP_INFO(this->get_logger(), "%s", info.str().c_str());

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in decryption: %s", e.what());
    }
  }

  // 기존 터틀 플랜트 콜백
  void turtle1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg){
    turtle1_state_pub_->publish(*msg);
  }
  void turtle2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg){
    turtle2_state_pub_->publish(*msg);
  }
  void control_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    turtle2_cmd_pub_->publish(*msg);
  }

  // ===== 상태 =====
  CryptoContext<DCRTPoly> cc;
  KeyPair<DCRTPoly> kp;
  bool emk_sent_ = false;

  int64_t SCALE_XY_   = 10;
  int64_t SCALE_SUM_  = 10;
  int64_t SCALE_PROD_ = 100;

  // 시간/로그
  std::chrono::high_resolution_clock::time_point loop_start_time_;
  std::chrono::high_resolution_clock::time_point encryption_start_;
  std::chrono::high_resolution_clock::time_point serialization_start_;
  double original_x_, original_y_;

  // ROS2 pub/sub
  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr emk_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr result_sub_;
  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr encrypted_pub_;

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_command_sub_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr turtle1_state_pub_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr turtle2_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_cmd_pub_;
  rclcpp::TimerBase::SharedPtr setup_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncTurtlePlant>());
  rclcpp::shutdown();
  return 0;
}
