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

    // === Setup 구독자, evalkey 받고 구독 해제할 예정 ===
    sub_emk_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
      "fhe_evalmult", qos,
      std::bind(&EncTurtleController::on_evalmult, this, std::placeholders::_1));

    // === 런타임 채널 === 구독 1개 퍼블리시 1개
    encrypted_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
      "encrypted_pose", qos,
      std::bind(&EncTurtleController::enc_pose_callback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>("encrypted_result", qos);
  }

private:
  // --- EvalMultKey 수신 (cc는 암호문에서 획득하므로 도착 전까지 버퍼링만) ---
  void on_evalmult(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg){
    std::string s(msg->data.begin(), msg->data.end());
    // 컨텍스트가 이미 있으면 즉시 등록, 아니면 버퍼링
    if (cntr_cc_ && !emk_registered_) {
      std::stringstream ss(s);
      cntr_cc_->DeserializeEvalMultKey(ss, SerType::BINARY);
      emk_registered_ = true;
      got_emk_ = true;
      pending_emk_.clear();
      if (sub_emk_) {
        sub_emk_.reset();
        RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult subscription closed (one-shot)");
      }
      RCLCPP_INFO(this->get_logger(), "[Controller] EvalMult key registered on ciphertext context (immediate)");
      return;
    }

    // 컨텍스트가 아직 없으면 버퍼만 하고, 생성 시 1회 등록 시도
    pending_emk_ = std::move(s);
    RCLCPP_INFO(this->get_logger(), "[Controller] EvalMultKey buffered (will register when context is ready)");
  }

  // --- 암호문 수신/연산 ---
  void enc_pose_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      std::string data_str(msg->data.begin(), msg->data.end());
      std::stringstream ss(data_str);

      if (msg->data_type == 0) { // X
        Serial::Deserialize(ciphertext_x_, ss, SerType::BINARY);
        got_x_ = true;

        // 첫 암호문에서 cc 얻음
        if (!cntr_cc_) {
          cntr_cc_ = ciphertext_x_->GetCryptoContext();
          cntr_cc_->Enable(KEYSWITCH);
          cntr_cc_->Enable(LEVELEDSHE);
          // 버퍼된 evalmult 키 등록 1회 시도
          if (!pending_emk_.empty() && !emk_registered_) {
            std::stringstream ess(pending_emk_);
            cntr_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
            emk_registered_ = true;
            got_emk_ = true;
            pending_emk_.clear();
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
          if (!pending_emk_.empty() && !emk_registered_) {
            std::stringstream ess(pending_emk_);
            cntr_cc_->DeserializeEvalMultKey(ess, SerType::BINARY);
            emk_registered_ = true;
            got_emk_ = true;
            pending_emk_.clear();
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
  std::string pending_emk_;                    // run_cc_ 준비 전 도착한 evalmult 키 버퍼

  Ciphertext<DCRTPoly> ciphertext_x_, ciphertext_y_;
  bool got_x_ = false, got_y_ = false;

  // ROS2
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
