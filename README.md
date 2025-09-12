<img width="1110" height="334" alt="image" src="https://github.com/user-attachments/assets/0120d299-8c1d-467d-9839-f5fb2d99c1fc" />
<rqt_graph >

<enc_turtle_plant>:

암호문 연산을 위한 evalmult를 초기화 단계에서 송신
터틀 봇 1의 pose 데이터 (x,y)를 받아 암호화 후 송신
암호화 된 연산 결과 x+y, x*y 를 받아 복호화 후 프린트

<enc_turtle_controller>
 
암호문 연산을 위한 cc(CipherContext)를 첫 암호문에서 추출 후 evalkey 등록
암호화 된 데이터 x,y 를 받아 연산 후 송신



<img width="1208" height="323" alt="image" src="https://github.com/user-attachments/assets/0d76de63-8d77-4681-97f1-e3406dde2c86" />
링 차원 : 약 8000 
암호화: 5ms
직렬화: 0.1ms
역직렬화: 1.5ms
복호화: 0.8ms
덧셈: 0.15ms
곱셈: 3.27ms
전체 루프: 15ms
통신시간: 약 4~5ms (전체 루프에서 위 과정의 차이로 추정)

# 현재 상황
<9/12>
BGV의 덧셈 곱셈 한번 정도의 연산 
1. 암호 컨텍스트 cc는 첫 송신받은 암호문에서 뽑아서 사용
2. Evalkey는 cc안에서 SerializeEvalMultkey 라는 함수가 존재하는 것으로 보아 시리얼화해서 통신하고 다시 등록하는게 맞는 과정으로 보임
3. PlaintextModulus 크기와 실수를 정수로 바꾸는 SCALE_XY_ 변수를 고려하여 설정이후 곱셈 깊이와 RingDim 보안 레벨 설정


이때 xml 파일로 QoS 설정을 통해 fast DDS의 데이터 max size를 2MB로 변경하여 사용

~~암호문을 연산하기 위한 evalkey가
plant에서 암호화를 하기 위해 만들었던 컨텍스트와
controller에서 생성한 컨텍스트가 다르다는 이유로 덧셈 연산만 수행할 수 있는 오류
따라서 OpenFHE는 evalkey를 플랜트가 던져줘야될 것으로 보임...
현재 : 
암호화 된 터틀봇 1의 x,y 값을 보냄 => 컨트롤러가 x+y 계산 후 전송 => 플랜트가 복호화 후 프린트
암호화 -> cereal 라이브러리를 통한 직렬화 -> string msg 을 통해 통신 
(현재 P = 65537 일때, N = 16384 (2^14) 정도이며 이때 데이터 크기는 1050103 약 1MB 사이즈)~~

# install
```
$ mkdir my_ws/  
$ cd ~/my_ws/
$ git clone https://github.com/lsw23101/Encrypted_turtle

$ colcon build --symlink-install
혹은
$ cba
```


# Requirement
ros2 (현:foxy)
turtlesim package
Openfhe 설치...(openfhe 환경 설정이 조금 어려웠습니다..)

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

[1] https://github.com/roboticvedant/ROS2_turtlesim_PID_demo



// 
****



#### git 다루기
https://shortcuts.tistory.com/8

#### OpenFHE의 scheme 테스트, 디버그 메모...

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
sangwon@STEIECDSL-P04:~/ROS2_turtlesim_ws/install/enc_turtle_cpp/lib/enc_turtle_cpp$ ./bgv_test
```


// N 사이즈가 2^13 일때 뎁스: 1

<img width="400" height="200" alt="image" src="https://github.com/user-attachments/assets/931f0fdd-07e8-4626-a2b3-fceb73d74fc5" />


// N 사이즈 2^12 일때 뎁스: 0 일때 65537 플레인 텍스트 크기에 대해서 이게 마지노선

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/780c3537-c846-4351-b90b-a6b4ba0f4394" />

// 
****
