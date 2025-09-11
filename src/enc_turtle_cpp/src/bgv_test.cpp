#include "openfhe/pke/openfhe.h"
#include <chrono>
#include <iomanip>
#include <iostream>
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

int main() {
    // 파라미터 설정
    CCParams<CryptoContextBGVRNS> parameters;
    parameters.SetPlaintextModulus(65537);
    parameters.SetMultiplicativeDepth(0);  // 덧셈만 사용하므로 0으로 설정
    parameters.SetSecurityLevel(SecurityLevel::HEStd_128_classic);
    parameters.SetRingDim(4096);  // 2^13으로 줄임

    std::cout << "========== BGV 암호화 성능 테스트 시작 ==========\n";
    
    // 컨텍스트 생성 시간 측정
    auto context_start = std::chrono::high_resolution_clock::now();
    auto cc = GenCryptoContext(parameters);
    cc->Enable(PKE);
    cc->Enable(LEVELEDSHE);
    cc->Enable(ADVANCEDSHE);
    auto context_end = std::chrono::high_resolution_clock::now();
    auto context_time = std::chrono::duration_cast<std::chrono::microseconds>(
        context_end - context_start).count() / 1000.0;
    std::cout << "컨텍스트 생성 시간: " << context_time << " ms\n";

    // 키 생성 시간 측정
    auto keygen_start = std::chrono::high_resolution_clock::now();
    auto keyPair = cc->KeyGen();
    cc->EvalMultKeyGen(keyPair.secretKey);
    cc->EvalSumKeyGen(keyPair.secretKey);
    auto keygen_end = std::chrono::high_resolution_clock::now();
    auto keygen_time = std::chrono::duration_cast<std::chrono::microseconds>(
        keygen_end - keygen_start).count() / 1000.0;
    std::cout << "키 생성 시간: " << keygen_time << " ms\n\n";

    // 테스트 데이터 준비
    std::vector<int64_t> x = {5544};  // ROS에서 사용한 것과 비슷한 값
    std::vector<int64_t> y = {5544};

    // 암호화 시간 측정
    auto enc_start = std::chrono::high_resolution_clock::now();
    auto plaintextX = cc->MakePackedPlaintext(x);
    auto plaintextY = cc->MakePackedPlaintext(y);
    auto ciphertextX = cc->Encrypt(keyPair.publicKey, plaintextX);
    auto ciphertextY = cc->Encrypt(keyPair.publicKey, plaintextY);
    auto enc_end = std::chrono::high_resolution_clock::now();
    auto enc_time = std::chrono::duration_cast<std::chrono::microseconds>(
        enc_end - enc_start).count() / 1000.0;
    std::cout << "암호화 시간 (두 개의 값): " << enc_time << " ms\n";

    // 직렬화 시간 측정
    auto ser_start = std::chrono::high_resolution_clock::now();
    std::stringstream ss_x, ss_y;
    Serial::Serialize(ciphertextX, ss_x, SerType::BINARY);
    Serial::Serialize(ciphertextY, ss_y, SerType::BINARY);
    auto ser_end = std::chrono::high_resolution_clock::now();
    auto ser_time = std::chrono::duration_cast<std::chrono::microseconds>(
        ser_end - ser_start).count() / 1000.0;
    std::cout << "직렬화 시간 (두 개의 값): " << ser_time << " ms\n";

    // 직렬화된 데이터 크기 확인
    std::string x_str = ss_x.str();
    std::string y_str = ss_y.str();
    std::cout << "직렬화된 데이터 크기: " << x_str.size() << " bytes\n\n";

    // 역직렬화 시간 측정
    auto deser_start = std::chrono::high_resolution_clock::now();
    Ciphertext<DCRTPoly> restoredX;
    Ciphertext<DCRTPoly> restoredY;
    Serial::Deserialize(restoredX, ss_x, SerType::BINARY);
    Serial::Deserialize(restoredY, ss_y, SerType::BINARY);
    auto deser_end = std::chrono::high_resolution_clock::now();
    auto deser_time = std::chrono::duration_cast<std::chrono::microseconds>(
        deser_end - deser_start).count() / 1000.0;
    std::cout << "역직렬화 시간 (두 개의 값): " << deser_time << " ms\n";

    // 복호화 시간 측정
    auto dec_start = std::chrono::high_resolution_clock::now();
    Plaintext decryptedX;
    Plaintext decryptedY;
    cc->Decrypt(keyPair.secretKey, restoredX, &decryptedX);
    cc->Decrypt(keyPair.secretKey, restoredY, &decryptedY);
    decryptedX->SetLength(1);
    decryptedY->SetLength(1);
    auto dec_end = std::chrono::high_resolution_clock::now();
    auto dec_time = std::chrono::duration_cast<std::chrono::microseconds>(
        dec_end - dec_start).count() / 1000.0;
    std::cout << "복호화 시간 (두 개의 값): " << dec_time << " ms\n";

    // 결과 확인
    std::cout << "\n복호화된 결과:\n";
    std::cout << "X: " << decryptedX->GetPackedValue()[0] << "\n";
    std::cout << "Y: " << decryptedY->GetPackedValue()[0] << "\n";

    // 전체 시간 계산
    auto total_time = context_time + keygen_time + enc_time + ser_time + deser_time + dec_time;
    std::cout << "\n========== 성능 요약 ==========\n";
    std::cout << "컨텍스트 생성: " << context_time << " ms (" 
              << std::fixed << std::setprecision(1) << (context_time/total_time)*100 << "%)\n";
    std::cout << "키 생성: " << keygen_time << " ms (" 
              << (keygen_time/total_time)*100 << "%)\n";
    std::cout << "암호화: " << enc_time << " ms (" 
              << (enc_time/total_time)*100 << "%)\n";
    std::cout << "직렬화: " << ser_time << " ms (" 
              << (ser_time/total_time)*100 << "%)\n";
    std::cout << "역직렬화: " << deser_time << " ms (" 
              << (deser_time/total_time)*100 << "%)\n";
    std::cout << "복호화: " << dec_time << " ms (" 
              << (dec_time/total_time)*100 << "%)\n";
    std::cout << "총 시간: " << total_time << " ms (100%)\n";
    std::cout << "==============================\n";

    return 0;
} 