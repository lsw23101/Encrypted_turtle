#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "enc_turtle_cpp/msg/encrypted_data.hpp"
#include "openfhe/pke/openfhe.h"
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>

// cereal 등록은 src/openfhe_cereal_registration.cpp 한 곳에서 처리

using namespace lbcrypto;

//노드 클래스
class EncTurtlePlant : public rclcpp::Node {
public:
  // 생성자
  EncTurtlePlant() : Node("enc_turtle_plant") {
    // ===== 스케일 상수 =====
    SCALE_XY_   = 50;                   // x,y 스케일 스케일한 연산 결과가 PlaintextmModulus/2 를 넘지 않도록
    SCALE_SUM_  = SCALE_XY_;            // 덧셈 스케일
    SCALE_PROD_ = SCALE_XY_ * SCALE_XY_; // 곱셈 스케일

    // ===== 암호화 설정 =====
    CCParams<CryptoContextBGVRNS> parameters;
    // parameters.SetPlaintextModulus(65537);
    parameters.SetPlaintextModulus(557057); // 검색해서 ntt 소수 적당한 것 찾기...
    parameters.SetMultiplicativeDepth(1);       // x*y 1회
    parameters.SetSecurityLevel(SecurityLevel::HEStd_NotSet);
    parameters.SetRingDim(8192); // NotSet이라

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

    // 처음 세팅에 쓰이는 객체, evalkey 송신 이후 사용 x
    emk_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>("fhe_evalmult", qos);
    // 타이머 객체
    setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                    [this]{ send_evalmult_once(); });
    
    
    // ===== 런타임 채널 =====
    // 구독자 두개 만들기 
    // 터틀봇 pose 받음
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", qos,
        std::bind(&EncTurtlePlant::pose_callback, this, std::placeholders::_1));

    // 연산 결과 받음
    result_sub_ = this->create_subscription<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_result", qos,
        std::bind(&EncTurtlePlant::result_callback, this, std::placeholders::_1));

    // 암호화 데이터 보낼 퍼블리셔
    encrypted_pub_ = this->create_publisher<enc_turtle_cpp::msg::EncryptedData>(
        "encrypted_pose", qos);
  }

private:
  // 멤버함수
  // cc 내부 함수로 EvalMultKey 직렬화 -> msg_emk 퍼블리시
  void send_evalmult_once(){
    if (emk_sent_) return;
    emk_sent_ = true;

    std::stringstream ss_emk;
    cc->SerializeEvalMultKey(ss_emk, SerType::BINARY);
    std::string str_emk = ss_emk.str();

    enc_turtle_cpp::msg::EncryptedData msg_emk;
    msg_emk.data.assign(str_emk.begin(), str_emk.end());
    emk_pub_->publish(msg_emk);

    RCLCPP_INFO(this->get_logger(), "[Plant] Setup sent: EvalMultKey only");
  }

  // pose를 받으면 콜백 -> 받은 데이터 암호화 직렬화 송신 
  // turtle1_state_pub_ 퍼블리시하고 있지만 현재는 사용 x
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

      // 직렬화 (바이트스트림으로 바로 변환)
      serialization_start_ = std::chrono::high_resolution_clock::now();
      std::stringstream ss_x, ss_y;
      Serial::Serialize(ciphertext_x, ss_x, SerType::BINARY);
      Serial::Serialize(ciphertext_y, ss_y, SerType::BINARY);

      // stringstream → vector<uint8_t> 직접 변환
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
           << ", y=" << original_y_ << " (결과: " << original_x_ + original_y_ << " x*y: " << original_x_ * original_y_ << ")\n"
           << "암호화 시간: " << encryption_time << " ms\n"
           << "직렬화 시간: " << serialization_time << " ms\n"
           << "데이터 크기(x,y): " << x_data.size() << " " << y_data.size() << " bytes\n"
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

      // 상태 퍼블리시 제거 (enc_turtle_controller에서 사용하지 않음)

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in encryption: %s", e.what());
    }
  }

  // 결과 수신(덧셈/곱셈 모두 처리: 합은 ÷SCALE_XY, 곱은 ÷(SCALE_XY^2))
  // result_sub_가 암호화된 연산 결과를 받으면 수행하는 콜백 함수
  void result_callback(const enc_turtle_cpp::msg::EncryptedData::SharedPtr msg) {
    try {
      // 시간 측정
      Ciphertext<DCRTPoly> ct;
      double deser_time_ms = 0.0;
      {
        auto deser_start = std::chrono::high_resolution_clock::now();
        // 바이트스트림을 직접 stringstream으로 변환
        std::stringstream ss(std::string(msg->data.begin(), msg->data.end()));
        Serial::Deserialize(ct, ss, SerType::BINARY);
        auto deser_end = std::chrono::high_resolution_clock::now();
        deser_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
          deser_end - deser_start).count() / 1000.0;
      }

      // 암호문 전달받은 데이터 저장
      if (msg->data_type == 2) { // x+y
        ct_sum_buf_ = ct;
        got_sum_ = true;
      } else if (msg->data_type == 3) { // x*y
        ct_prod_buf_ = ct;
        got_prod_ = true;
      } else {
        return; // 알 수 없는 타입은 무시
      }

      // 두개 받고 나서 복호화
      if (got_sum_ && got_prod_) {
        auto dec_start = std::chrono::high_resolution_clock::now();

        // (x+y)
        Plaintext pt_sum;
        cc->Decrypt(kp.secretKey, ct_sum_buf_, &pt_sum);
        pt_sum->SetLength(1); // 패킹과 관련하여 사용, 현재는 스칼라 데이터 암호화하여 길이 1
        double sum_val = double(pt_sum->GetPackedValue()[0]) / double(SCALE_SUM_); // 스케일 복원

        // (x*y)
        Plaintext pt_prod;
        cc->Decrypt(kp.secretKey, ct_prod_buf_, &pt_prod);
        pt_prod->SetLength(1);
        double prod_val = double(pt_prod->GetPackedValue()[0]) / double(SCALE_PROD_); // 스케일 복원

        // 시간 측정
        auto dec_end = std::chrono::high_resolution_clock::now();
        double decryption_time = std::chrono::duration_cast<std::chrono::microseconds>(
          dec_end - dec_start).count() / 1000.0;

        // 위에서 측정한 현재 메시지의 역직렬화 시간 사용
        double deserialization_time = deser_time_ms;

        // 전체 루프 시간: pose 콜백에서 기록한 loop_start_time_ 기준
        double total_loop_time = std::chrono::duration_cast<std::chrono::microseconds>(
          dec_end - loop_start_time_).count() / 1000.0;

        RCLCPP_INFO(this->get_logger(),
          "\n========== 암호화 통신 결과 ==========\n"
          "Dec(Enc(x)+Enc(y)) = %.3f,   Dec(Enc(x)*Enc(y)) = %.3f\n"
          "역직렬화 시간: %.3f ms\n"
          "복호화 시간: %.3f ms\n"
          "전체 루프 시간: %.3f ms\n"
          "===================================================",
          sum_val, prod_val, deserialization_time, decryption_time, total_loop_time);

        // 루프 초기화
        got_sum_ = got_prod_ = false;
        ct_sum_buf_.reset();
        ct_prod_buf_.reset();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in result handling: %s", e.what());
    }
  }

  // 멤버변수
  CryptoContext<DCRTPoly> cc;
  KeyPair<DCRTPoly> kp;
  bool emk_sent_ = false;
  bool got_sum_ = false, got_prod_ = false;
  Ciphertext<DCRTPoly> ct_sum_buf_, ct_prod_buf_;

  int64_t SCALE_XY_;
  int64_t SCALE_SUM_;
  int64_t SCALE_PROD_;

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

  
  
  rclcpp::TimerBase::SharedPtr setup_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncTurtlePlant>());
  rclcpp::shutdown();
  return 0;
}
