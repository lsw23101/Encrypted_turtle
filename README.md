<img width="994" height="160" alt="image" src="https://github.com/user-attachments/assets/3d84cdda-2584-4e9a-b449-fc3b0f58f5c2" />  

<rqt_graph >

# 내용
1. 플랜트 <enc_turtle_plant>: 터틀봇의 위치 (x,y) 를 암호화 후 송신
2. 컨트롤러 <enc_turtle_controller>: 암호문 데이터 **x**, **y**의 덧셈, 곱셈 1회 수행 후 송신
3. 이때 암호문은 cereal을 이용하여 바이트스트림으로 변환 후 통신


# 결과
1. BGV의 덧셈 곱셈 한번 정도의 연산
2. 암호문간 연산을 위해서는 암호 컨텍스(CryptoContext, cc)가 필요하며 이 cc에 연산 키(EvalKey)가 필요
3. 연산이 수행되는 암호문은 암호화 될 때와 "동일한" cc를 이용하여 연산이 되어야 함
4. EvalKey와 같은 연산 키는 비밀키를 기반으로 생성하므로 컨트롤러에서 만들 수 없음
   (추측, Lattigo에서는 sk 없이 evalkey 생성 가능 했던 것으로 기억)
6. 플랜트에서 생성한 cc를 직렬화 후 통신하면 오류 발생 -> 첫 암호문에서 cc 추출 (cc를 뽑아내는 함수가 존재)
7. 플랜트에서 생성한 EvalKey는 직렬화 후 통신하여 컨트롤러가 5에서 뽑아낸 cc에 "등록"
8. 실수 데이터를 정수화 하기 위한 스케일 상수(SCALE_XY_)에 맞는 평문 공간(PlaintextModulus) 설정 필요
9. 현재 보안 레벨은 NotSet


src/enc_turtle_cpp/config/fastdds_config.xml
이때 xml 파일로 QoS 설정을 통해 fast DDS의 데이터 max size를 2MB로 변경하여 사용

<img width="1208" height="323" alt="image" src="https://github.com/user-attachments/assets/0d76de63-8d77-4681-97f1-e3406dde2c86" />  
링 차원 : 약 8000  
암호화: 5ms  
직렬화: 0.1ms  
역직렬화: 1.5ms  
복호화: 0.8ms  
덧셈: 0.15ms  
곱셈: 3.27ms  
전체 루프: 15ms  
통신시간: 약 4~5ms (전체 루프에서 위 과정의 차이로 추정) (통신이 다른 pc가 되거나 무선이 되는 등 상황이면 더 커질 것으로 예상)

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
cereal 라이브러리 (openfhe의 종속성으로 같이 설치)

# Usage
1. 배쉬 실행
```
$ source /opt/ros/foxy/setup.bash
$ source install/setup.bash

OR 단축어 설정시

$ rosfoxy 
```

2. 런치파일 실행

암호문 전송 연산 데모
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
 cd ~/ROS2_turtlesim_ws && colcon build --packages-select enc_turtle_cpp && source install/setup.bash && ros2 run enc_turtle_cpp bgv_test

```

(암호 보안 레벨 비설정 가능)
openfhecore/include/lattice/params.h
```
parameters.SetSecurityLevel(SecurityLevel::HEStd_NotSet); // 자동 결정 방지
```

#### open fhe scheme 속도

```
~/ROS2_turtlesim_ws/install/enc_turtle_cpp/lib/enc_turtle_cpp$ ./bgv_test
```


// N 사이즈가 2^13 일때 뎁스: 1

<img width="400" height="200" alt="image" src="https://github.com/user-attachments/assets/931f0fdd-07e8-4626-a2b3-fceb73d74fc5" />


// N 사이즈 2^12 일때 뎁스: 0 일때 65537 플레인 텍스트 크기에 대해서 이게 마지노선

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/780c3537-c846-4351-b90b-a6b4ba0f4394" />

// 
****
</details>
