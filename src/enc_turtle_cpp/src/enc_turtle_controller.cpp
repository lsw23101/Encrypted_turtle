#include "rclcpp/rclcpp.hpp"
#include "enc_turtle_cpp/msg/encrypted_data.hpp"
#include "openfhe/pke/openfhe.h"
#include <thread>
#include <cmath>
#include <sstream>
#include <chrono>

// cereal 등록은 src/openfhe_cereal_registration.cpp 한 곳에서 처리

using namespace lbcrypto;

class EncTurtleController : public rclcpp::Node {
public:
  // 생성자
  EncTurtleController() : Node("enc_turtle_controller") {
    // QoS
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    // 처음 세팅 evalkey 구독
    sub_emk_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
      "fhe_evalmult", qos,
      std::bind(&EncTurtleController::on_evalmult, this, std::placeholders::_1));

    // === 런타임 채널 === 암호 데이터 구독 / 연산 결과 퍼블리시
    encrypted_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
      "encrypted_pose", qos,
      std::bind(&EncTurtleController::enc_pose_callback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>("encrypted_result", qos);
  }

private:
  // --- EvalMultKey 수신하는 함수 (cc는 암호문에서 획득하므로 도착 전까지 버퍼링만) ---
  void on_evalmult(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg){
    // 1. 수신한 메시지에서 바이트 데이터를 std::string으로 변환
    std::string s(msg->data.begin(), msg->data.end());
    
    // 2. 컨텍스트가 이미 준비되어 있고 아직 키가 등록되지 않았는지 확인
    if (cntr_cc_ && !emk_registered_) {
      // 3. 바이트 데이터를 stringstream으로 변환 (역직렬화용)
      std::stringstream ss(s);
      // 4. 컨텍스트에 EvalMultKey 등록 (바이너리 형식으로 역직렬화)
      cntr_cc_->DeserializeEvalMultKey(ss, SerType::BINARY);
      // 5. 키 등록 완료 플래그들 설정
      emk_registered_ = true;
      got_emk_ = true;
      // 6. 대기 중인 키 데이터 정리 (이미 등록했으므로 불필요)
      waiting_emk_.clear();
      
      // 7. 1회성 구독이므로 등록 완료 후 구독 해제
      if (sub_emk_) {
        sub_emk_.reset();
        RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult subscription closed (one-shot)");
      }
      // 8. 등록 완료 로그 출력
      RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context (immediate)");
      return; // 9. 즉시 등록 완료했으므로 함수 종료
    }

    // 10. 컨텍스트가 아직 준비되지 않았거나 이미 등록된 경우
    // 11. 키 데이터를 waiting_emk_ 버퍼에 저장 (move로 효율적 전달)
    waiting_emk_ = std::move(s);
    // 12. 대기 중 저장 완료 로그 출력 (컨텍스트 준비되면 나중에 등록 예정)
    RCLCPP_INFO(this->get_logger(), "[Controller] EvalMultKey waiting (will register when context is ready)");
  }

  // 암호문 수신 후 콜백 함수
  void enc_pose_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      // 바이트스트림을 직접 stringstream으로 변환
      std::stringstream ss(std::string(msg->data.begin(), msg->data.end()));

      if (msg->data_type == 0) { // X
        Serial::Deserialize(ciphertext_x_, ss, SerType::BINARY);
        got_x_ = true;

        // 첫 암호문에서 cc 얻음
        if (!cntr_cc_) {
          cntr_cc_ = ciphertext_x_->GetCryptoContext();
          cntr_cc_->Enable(KEYSWITCH);
          cntr_cc_->Enable(LEVELEDSHE);
          // 대기 중인 evalmult 키 등록 1회 시도
          if (!waiting_emk_.empty() && !emk_registered_) {
            std::stringstream ess(waiting_emk_);
            cntr_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
            emk_registered_ = true;
            got_emk_ = true;
            waiting_emk_.clear();
            if (sub_emk_) {
              sub_emk_.reset();
              RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult subscription closed (one-shot)");
            }
            RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context");
          }
        }
        RCLCPP_INFO(this->get_logger(), "Encrypted X received");
      
      // cc 얻은 이후 암호문 받아서 연산
      } else if (msg->data_type == 1) { // Y
        Serial::Deserialize(ciphertext_y_, ss, SerType::BINARY);
        got_y_ = true;

        if (!cntr_cc_) {
          cntr_cc_ = ciphertext_y_->GetCryptoContext();
          cntr_cc_->Enable(KEYSWITCH);
          cntr_cc_->Enable(LEVELEDSHE);
          if (!waiting_emk_.empty() && !emk_registered_) {
            std::stringstream ess(waiting_emk_);
            cntr_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
            emk_registered_ = true;
            got_emk_ = true;
            waiting_emk_.clear();
            if (sub_emk_) {
              sub_emk_.reset();
              RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult subscription closed (one-shot)");
            }
            RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context");
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

        // 여기서는 늦은 등록을 시도하지 않음(컨텍스트 획득 시 1회만 등록)

        if (!got_emk_) {
          RCLCPP_WARN(this->get_logger(), "[Controller] EvalMult key not registered yet; skipping mult");
        }

        // === 같은 컨텍스트(cntr_cc_)로 연산 ===
        // 덧셈 시간 측정
        auto add_start = std::chrono::high_resolution_clock::now();
        
        // EvalAdd는 Evalkey 없어도 가능해보임
        auto ct_sum  = cntr_cc_->EvalAdd(ciphertext_x_, ciphertext_y_);
        auto add_end = std::chrono::high_resolution_clock::now();
        double add_ms = std::chrono::duration_cast<std::chrono::microseconds>(add_end - add_start).count() / 1000.0;
        enc_turtle_cpp::msg::EncryptedData out_sum;
        {
          std::stringstream ss_out;
          Serial::Serialize(ct_sum, ss_out, SerType::BINARY);
          auto s = ss_out.str();
          out_sum.data.assign(s.begin(), s.end());
          out_sum.data_type = 2; // x+y
          result_pub_->publish(out_sum);
        }
        
        // EvalMult는 Evalkey 있어야 가능
        if (got_emk_) {
          // 곱셈 시간 측정
          auto mul_start = std::chrono::high_resolution_clock::now();
          auto ct_prod = cntr_cc_->EvalMult(ciphertext_x_, ciphertext_y_);
          auto mul_end = std::chrono::high_resolution_clock::now();
          double mul_ms = std::chrono::duration_cast<std::chrono::microseconds>(mul_end - mul_start).count() / 1000.0;
          enc_turtle_cpp::msg::EncryptedData out_prod;
          std::stringstream ss_out2;
          Serial::Serialize(ct_prod, ss_out2, SerType::BINARY);
          auto s2 = ss_out2.str();
          out_prod.data.assign(s2.begin(), s2.end());
          out_prod.data_type = 3; // x*y
          result_pub_->publish(out_prod);
          RCLCPP_INFO(this->get_logger(), "[Controller] EvalAdd: %.3f ms, EvalMult: %.3f ms", add_ms, mul_ms);
        }
        
        // // Evalkey 못받았을 때 
        // if (!got_emk_) {
        //   RCLCPP_INFO(this->get_logger(), "[Controller] EvalAdd: %.3f ms (EvalMult skipped: no eval key)", add_ms);
        // }
        // RCLCPP_INFO(this->get_logger(), "Published enc (x+y)%s",
        //   got_emk_ ? " and (x*y)" : " (mult skipped: no eval key)");
        // 다음 프레임을 위해 반드시 리셋
        got_x_ = false;
        got_y_ = false;
        ciphertext_x_.reset();
        ciphertext_y_.reset();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in homomorphic ops: %s", e.what());
    }
  }


  // 상태
  CryptoContext<DCRTPoly> cntr_cc_;             // 실제 연산 컨텍스트(암호문에서 획득)
  bool got_emk_ = false;
  bool emk_registered_ = false;
  std::string waiting_emk_;                    // 컨텍스트 준비 전 도착한 evalmult 키 대기 버퍼

  Ciphertext<DCRTPoly> ciphertext_x_, ciphertext_y_;
  bool got_x_ = false, got_y_ = false;

  // ROS2 pub/sub 3개 1개는 초기 설정 2개는 런타임
  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr sub_emk_;
  rclcpp::Subscription<enc_turtle_cpp::msg::EncryptedData>::SharedPtr encrypted_sub_;
  rclcpp::Publisher<enc_turtle_cpp::msg::EncryptedData>::SharedPtr result_pub_;
  
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncTurtleController>());
  rclcpp::shutdown();
  return 0;
}
