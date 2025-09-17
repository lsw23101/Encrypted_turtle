<img width="994" height="160" alt="image" src="https://github.com/user-attachments/assets/3d84cdda-2584-4e9a-b449-fc3b0f58f5c2" />  

<rqt_graph >

# 내용
1. 플랜트 <enc_turtle_plant>: 터틀봇의 위치 (x,y) 를 암호화 후 송신
2. 컨트롤러 <enc_turtle_controller>: 암호문 데이터 **x**, **y**의 덧셈, 곱셈 1회 수행 후 송신
3. 이때 암호문은 cereal을 이용하여 string으로 변환 후 uint8[]으로 통신 (ROS에서 char 통신은 안되는 것으로 보임)


# 결과
1. BGV의 덧셈 곱셈 한번 정도의 연산
2. 암호문간 연산을 위해서는 암호 컨텍스(CryptoContext, cc)가 필요하며 이 cc에 연산 키(EvalKey) 등록 필요
3. 연산이 수행되는 암호문은 암호화 될 때와 "동일한" cc를 이용하여 연산이 되어야 함
4. EvalKey와 같은 연산 키는 비밀키를 기반으로 생성하므로 컨트롤러에서 만들 수 없음  
   (Lattigo에서 eval key는 sk가 없어도 되지만, 재선형화키는 sk가 필요)
   (OpenFHE는 곱셈 계산 키가 재선형화 키가 필요한 것으로 추측)
6. 플랜트에서 생성한 cc를 직렬화 후 통신하면 오류 발생 -> 첫 암호문에서 cc 추출 (cc를 뽑아내는 함수가 존재)
7. 플랜트에서 생성한 EvalKey는 직렬화 후 통신하여 컨트롤러가 5에서 뽑아낸 cc에 "등록"
8. 실수 데이터를 정수화 하기 위한 스케일 팩터(SCALE_XY_)에 맞는 평문 공간(PlaintextModulus) 설정 필요
9. 현재 스케일 팩터는 50, 평문 공간은 557057, 보안레벨 128-bit
10. src/enc_turtle_cpp/config/fastdds_config.xml 파일로 QoS 설정을 통해 fast DDS의 데이터 max size를 2MB로 변경하여 사용


<img width="1208" height="323" alt="image" src="https://github.com/user-attachments/assets/0d76de63-8d77-4681-97f1-e3406dde2c86" />  

링 차원 : 약 8000  
암호화: 5ms  
직렬화: 0.1ms  
역직렬화: 1.5ms  
복호화: 0.8ms  
덧셈: 0.15ms  
곱셈: 3.27ms  
전체 루프: 15ms  
암호화-복호화 과정이 약 10-11m 이므로 통신시간은 약 4-5ms 으로 추측  
(한 pc에서 돌아갈때 저 속도이므로 통신이 다른 pc가 되거나 무선이 되는 등 상황이면 더 커질 것으로 예상)

# 논의점...
1. 지금 방법(직렬화, 준비단계 등) 외의 이미 잘 짜여진 암호문의 통신 방법이 있을지...  
   (9/17 Lattigo에서는 통신 버퍼에 암호문을 바로 읽고 쓰는 함수가 구현되어있습니다.)
3. BGV가 아닌 CKKS에서는 안될지도 모르겠습니다..
4. 파라미터 설정 (스케일 팩터, 평문 공간, 링 차원 등등)



# Install
```
$ mkdir my_ws/  
$ cd ~/my_ws/
$ git clone https://github.com/lsw23101/Encrypted_turtle

$ colcon build --symlink-install
혹은
$ cba
```

# Requirement
ROS2 (현:foxy)  
turtlesim package  
OpenFHE 설치 (openfhe 환경 설정이 조금 안맞을 수도 있습니다...)  
cereal 라이브러리 (openfhe의 종속성으로 같이 설치되는 것 같습니다.)

# Usage
1. 배쉬 실행
```
$ source /opt/ros/foxy/setup.bash
$ source install/setup.bash

OR 단축어 설정시

$ rosfoxy 
```

2. 런치파일 실행

데모
```
$ ros2 launch enc_turtle_cpp enc_turtle_demo.launch.py
```

test_enc_turtle_demo.launch.py : 터틀봇 1과 2의 암호문을 받아 leader-follwer 해보려고 시도 중..  
turtle_demo.launch.py : [1]에서 제공하는 leader-follower 데모

런치파일 실행하면 teleop_twist_keyboard 에서 터틀봇 조종 가능


# Reference

[1] https://github.com/roboticvedant/ROS2_turtlesim_PID_demo (터틀봇 PID 예제)

****



<details>
 <summary>테스트, 디버그 메모...</summary>
 
#### git 다루기
https://shortcuts.tistory.com/8
 
# git push
```
$ git add src
$ git commit -m "message"
$ git push
```
bgv 테스트 용

```
 cd ~/Encrypted_turtle && colcon build --packages-select enc_turtle_cpp && source install/setup.bash && ros2 run enc_turtle_cpp bgv_test

```

(암호 보안 레벨 비설정 가능)
openfhecore/include/lattice/params.h
```
parameters.SetSecurityLevel(SecurityLevel::HEStd_NotSet); // 자동 결정 방지
```
# ntt 소수




#### open fhe scheme 속도

****
</details>
