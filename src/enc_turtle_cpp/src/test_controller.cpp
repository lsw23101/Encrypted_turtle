#include "rclcpp/rclcpp.hpp"
#include "enc_turtle_cpp/msg/encrypted_data.hpp"
#include "openfhe/pke/openfhe.h"
#include <thread>
#include <chrono>
#include <sstream>

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

class TestController : public rclcpp::Node {
public:
  TestController() : Node("test_controller") {
    // 암호화 설정 - Plant와 동일하게 설정
    CCParams<CryptoContextBGVRNS> parameters;
    parameters.SetPlaintextModulus(65537);
    parameters.SetMultiplicativeDepth(2);
    parameters.SetSecurityLevel(SecurityLevel::HEStd_128_classic);
    parameters.SetRingDim(16384);

    cc = GenCryptoContext(parameters);
    cc->Enable(PKE);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);

    // QoS 설정
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    // 암호화된 데이터 구독자
    encrypted_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_numbers", qos,
        std::bind(&TestController::encrypted_callback, this, std::placeholders::_1));

    // 결과 발행자
    result_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_result", qos);

    RCLCPP_INFO(this->get_logger(), "Waiting for keys...");
  }

private:
  void encrypted_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      std::string data_str(msg->data.begin(), msg->data.end());
      std::stringstream ss(data_str);

      if (msg->data_type == 2) {  // PublicKey
        RCLCPP_INFO(this->get_logger(), "Received PublicKey");
        PublicKey<DCRTPoly> pubKey;
        Serial::Deserialize(pubKey, ss, SerType::BINARY);
        public_key_ = pubKey;  // 저장만 하고 나중에 사용
        has_pubkey_ = true;
      }
      else if (msg->data_type == 3) {  // EvalMultKey
        RCLCPP_INFO(this->get_logger(), "Received EvalMultKey");
        std::vector<EvalKey<DCRTPoly>> evalMultKey;
        Serial::Deserialize(evalMultKey, ss, SerType::BINARY);
        cc->InsertEvalMultKey(evalMultKey);
        has_evalkey_ = true;
      }
      else if (msg->data_type == 0) {  // First number
        if (!has_pubkey_ || !has_evalkey_) {
          RCLCPP_WARN(this->get_logger(), "Received number but keys not ready");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Received first number");
        Serial::Deserialize(ciphertext1_, ss, SerType::BINARY);
        has_first_ = true;
      }
      else if (msg->data_type == 1) {  // Second number
        if (!has_pubkey_ || !has_evalkey_) {
          RCLCPP_WARN(this->get_logger(), "Received number but keys not ready");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Received second number");
        Serial::Deserialize(ciphertext2_, ss, SerType::BINARY);
        has_second_ = true;
      }

      // 모든 데이터가 준비되면 곱셈 수행
      if (has_pubkey_ && has_evalkey_ && has_first_ && has_second_) {
        auto ct_result = cc->EvalMult(ciphertext1_, ciphertext2_);
        RCLCPP_INFO(this->get_logger(), "Multiplication performed");

        // 결과 직렬화 및 전송
        std::stringstream ss_result;
        Serial::Serialize(ct_result, ss_result, SerType::BINARY);
        std::string result_str = ss_result.str();
        std::vector<uint8_t> result_data(result_str.begin(), result_str.end());

        auto msg_result = enc_turtle_cpp::msg::EncryptedData();
        msg_result.data = result_data;
        msg_result.data_type = 4;  // Result type
        result_pub_->publish(msg_result);
        RCLCPP_INFO(this->get_logger(), "Published result");

        // 상태 초기화
        has_first_ = false;
        has_second_ = false;
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in processing: %s", e.what());
    }
  }

  CryptoContext<DCRTPoly> cc;
  PublicKey<DCRTPoly> public_key_;  // PublicKey 저장
  Ciphertext<DCRTPoly> ciphertext1_;
  Ciphertext<DCRTPoly> ciphertext2_;
  bool has_pubkey_ = false;
  bool has_evalkey_ = false;
  bool has_first_ = false;
  bool has_second_ = false;

  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr encrypted_sub_;
  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr result_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestController>());
  rclcpp::shutdown();
  return 0;
} 