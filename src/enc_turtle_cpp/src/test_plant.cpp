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

class TestPlant : public rclcpp::Node {
public:
  TestPlant() : Node("test_plant") {
    // 암호화 설정
    CCParams<CryptoContextBGVRNS> parameters;
    parameters.SetPlaintextModulus(65537);
    parameters.SetMultiplicativeDepth(2);
    parameters.SetSecurityLevel(SecurityLevel::HEStd_128_classic);
    parameters.SetRingDim(16384);

    cc = GenCryptoContext(parameters);
    cc->Enable(PKE);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    kp = cc->KeyGen();

    if (kp.secretKey) {
        // 곱셈 키 생성
        cc->EvalMultKeyGen(kp.secretKey);
        
        // 키 생성 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 키가 생성되었는지 확인
        try {
            auto evalMultKey = cc->GetEvalMultKeyVector("");
            RCLCPP_INFO(this->get_logger(), "EvalMultKey generated successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate EvalMultKey: %s", e.what());
            return;
        }

        // QoS 설정
        rclcpp::QoS qos(10);
        qos.reliable();
        qos.durability_volatile();

        // 암호화된 데이터 발행자
        encrypted_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>(
            "encrypted_numbers", qos);

        // 결과 구독자
        result_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
            "encrypted_result", qos,
            std::bind(&TestPlant::result_callback, this, std::placeholders::_1));

        // 타이머 설정 (1초마다 실행)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestPlant::timer_callback, this));

        // 키 전송
        std::stringstream ss_pubkey;
        Serial::Serialize(kp.publicKey, ss_pubkey, SerType::BINARY);
        std::string pubkey_str = ss_pubkey.str();
        std::vector<uint8_t> pubkey_data(pubkey_str.begin(), pubkey_str.end());

        // EvalMultKey 가져오기 (이미 존재 확인됨)
        auto evalMultKey = cc->GetEvalMultKeyVector("");
        
        std::stringstream ss_evalkey;
        Serial::Serialize(evalMultKey, ss_evalkey, SerType::BINARY);
        std::string evalkey_str = ss_evalkey.str();
        std::vector<uint8_t> evalkey_data(evalkey_str.begin(), evalkey_str.end());

        // PublicKey 전송
        auto msg_pubkey = enc_turtle_cpp::msg::EncryptedData();
        msg_pubkey.data = pubkey_data;
        msg_pubkey.data_type = 2;  // PublicKey
        encrypted_pub_->publish(msg_pubkey);

        // EvalMultKey 전송
        auto msg_evalkey = enc_turtle_cpp::msg::EncryptedData();
        msg_evalkey.data = evalkey_data;
        msg_evalkey.data_type = 3;  // EvalMultKey
        encrypted_pub_->publish(msg_evalkey);

        RCLCPP_INFO(this->get_logger(), "Plant node initialized and keys published");

    } else {
        RCLCPP_ERROR(this->get_logger(), "Key generation failed!");
    }
  }

private:
  void timer_callback() {
    try {
      // 3과 5를 암호화
      std::vector<int64_t> num1 = {3};
      std::vector<int64_t> num2 = {5};

      auto plaintext1 = cc->MakePackedPlaintext(num1);
      auto plaintext2 = cc->MakePackedPlaintext(num2);

      auto ciphertext1 = cc->Encrypt(kp.publicKey, plaintext1);
      auto ciphertext2 = cc->Encrypt(kp.publicKey, plaintext2);

      // 직렬화
      std::stringstream ss1, ss2;
      Serial::Serialize(ciphertext1, ss1, SerType::BINARY);
      Serial::Serialize(ciphertext2, ss2, SerType::BINARY);

      std::string str1 = ss1.str();
      std::string str2 = ss2.str();

      std::vector<uint8_t> data1(str1.begin(), str1.end());
      std::vector<uint8_t> data2(str2.begin(), str2.end());

      // 첫 번째 숫자 전송
      auto msg1 = enc_turtle_cpp::msg::EncryptedData();
      msg1.data = data1;
      msg1.data_type = 0;  // First number
      encrypted_pub_->publish(msg1);

      // 두 번째 숫자 전송
      auto msg2 = enc_turtle_cpp::msg::EncryptedData();
      msg2.data = data2;
      msg2.data_type = 1;  // Second number
      encrypted_pub_->publish(msg2);

      RCLCPP_INFO(this->get_logger(), "Published encrypted numbers: 3 and 5");

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in encryption: %s", e.what());
    }
  }

  void result_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      std::string data_str(msg->data.begin(), msg->data.end());
      std::stringstream ss(data_str);
      
      Ciphertext<DCRTPoly> ciphertext;
      Serial::Deserialize(ciphertext, ss, SerType::BINARY);
      
      Plaintext plaintext;
      cc->Decrypt(kp.secretKey, ciphertext, &plaintext);
      plaintext->SetLength(1);
      
      int64_t result = plaintext->GetPackedValue()[0];
      RCLCPP_INFO(this->get_logger(), "Decrypted result: %ld", result);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in decryption: %s", e.what());
    }
  }

  CryptoContext<DCRTPoly> cc;
  KeyPair<DCRTPoly> kp;

  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr encrypted_pub_;
  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr result_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPlant>());
  rclcpp::shutdown();
  return 0;
} 