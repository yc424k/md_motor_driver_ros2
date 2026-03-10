# MD_controller (For Dual Channel motor driver)
This is a package that makes MDROBOT's motor driver available in ROS2(humble). [ https://www.mdrobot.co.kr ]

## 🔧 소개

본 레포지토리는 **MD 모터 드라이버**의 듀얼채널 모터를 제어하기 위한 **ROS 2 Humble** 기반 패키지입니다.
---

## ✅ 지원되는 모터 드라이버
- 본코드는 MDrobot의 듀얼채널 모터드라이버를 지원합니다.
- `md200t`
- `md400t`

---

## 🧪 테스트 환경 (실제 하드웨어 검증 완료 2025-07/30)

- `md200t` + `mdh80` with RS485(waveshare)
- `md400t` + `mdh250` with RS485(waveshare)

---

## 🧭 기능 설명

본 패키지는 ROS 2의 `/cmd_vel` 메시지를 받아 **좌/우 바퀴 RPM 제어**로 변환하여 MD 듀얼채널 모터 드라이버를 제어합니다.

- `/cmd_vel`의 **linear.x (전진/후진 속도)** 와 **angular.z (회전 속도)** 를 수신
- 내부적으로 이를 좌/우 바퀴 속도(RPM)로 변환
- 변환된 RPM을 MD 드라이버 명령으로 전송

### 4AWD (좌 A 드라이버 + 우 B 드라이버) 지원

현재 코드 기준으로 아래 구조를 지원합니다.

- 왼쪽(A) 드라이버: `ID`
- 오른쪽(B) 드라이버: `RightID`
- `/cmd_vel`로 계산된 좌/우 RPM을 각각 A/B 드라이버로 분리 전송
- `left_sign`, `right_sign`으로 방향 부호 보정
- `cmd_timeout_ms` 경과 시 자동 0 RPM 정지

> 중요: 현재 구현은 **단일 RS485 버스(단일 Port) + 드라이버 ID 분리** 구조 기준입니다.  
> 즉 A/B 드라이버가 서로 다른 물리 포트(`/dev/ttyUSB0`, `/dev/ttyUSB1`)인 경우는 별도 코드 확장이 필요합니다.

### 💻 동작 흐름 요약

/cmd_vel (linear.x, angular.z) <br>
↓ <br>
cmdVelToRpm() 변환 <br>
↓ <br>
left, right 모터 RPM 계산 <br>
↓ <br>
MD 모터 드라이버로 전송 <br>
↓ <br>
좌측/우측 바퀴 개별 회전 <br>

---

## 🔧 런치 파라미터 (launch/md_controller.launch.py)

하드웨어에 맞게 아래 파라미터를 수정하세요.

### 공통 파라미터

- `Port`: 직렬 포트 (기본 `/dev/ttyMotor`)
- `Baudrate`: 보드레이트 (기본 `57600`)
- `wheel_radius`: 바퀴 반지름(m)
- `wheel_base`: 좌우 바퀴 간 거리(m)
- `cmd_timeout_ms`: `cmd_vel` timeout 시 정지 시간(ms)
- `max_driver_rpm`: 드라이버 명령 rpm 제한

### 왼쪽(A) 드라이버 파라미터

- `ID`: 왼쪽 드라이버 ID
- `MDT`: 왼쪽 드라이버 target machine ID
- `GearRatio`: 왼쪽 기어비
- `poles`: 왼쪽 모터 pole 수
- `left_sign`: 방향 보정 (`1` 또는 `-1`)

### 오른쪽(B) 드라이버 파라미터

- `right_enabled`: 오른쪽 드라이버 사용 여부
- `RightID`: 오른쪽 드라이버 ID
- `RightMDT`: 오른쪽 드라이버 target machine ID
- `RightGearRatio`: 오른쪽 기어비
- `right_sign`: 방향 보정 (`1` 또는 `-1`)

<img width="411" height="355" alt="Screenshot from 2025-07-30 15-39-21" src="https://github.com/user-attachments/assets/16a82fda-5027-42b9-b966-627484fb38d7" />

---

## 📦 Dependencies
There is no official release of the serial package available for ROS2, so you need to install an unofficial version that is compatible with ROS2.

```
To install it, follow the steps below.

~$ git clone https://github.com/RoverRobotics-forks/serial-ros2.git
~$ cd serial-ros2
~$ mkdir build
~$ cd build
~$ cmake ..
~$ make
~$ cd ..
~$ colcon build --packages-select serial
~$ cd build
~$ sudo make install
```

## 🚀 실행방법

1. 모터 드라이버 런치 실행

```bash
ros2 launch md_controller md_controller.launch.py use_rviz:=False
```

2. (option) 키보드 teleop 테스트
> 단순히 `/cmd_vel`을 인가해 동작 확인할 때 사용합니다.

```bash
ros2 run md_teleop md_teleop_key_node
```

3. `/cmd_vel` 직접 발행 테스트 (저속 권장)

```bash
# very slow straight
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# very slow rotate
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

4. 방향 부호(sign) 보정

- 직진 명령에서 한쪽이 역방향이면 `left_sign` 또는 `right_sign`를 `-1`로 변경합니다.
- 두 드라이버가 다른 감속기/모터면 `GearRatio`와 `RightGearRatio`를 각각 맞추세요.

5. 안전 권장사항

- 처음에는 바퀴를 공중에 띄운 상태에서 테스트
- 저속에서 직진/회전/정지부터 확인
- E-stop을 반드시 활성화한 상태에서 주행 테스트

---

## 🤝 Contributors

| 이름            | GitHub ID | 소속                                         | 기여 내용                                 |
|-----------------|-----------|----------------------------------------------|-------------------------------------------|
| JungHo Cheon    | [c-jho](https://github.com/c-jho) | Korea Institute of Science and Technology | 원본 코드 작성 및 ROS2 포팅 기반 제공      |
| Seokgwon Lee    | [Lee-seokgwon](https://github.com/Lee-seokgwon)         | Kyungpook National Univ. (SEE)              | 듀얼채널 모터 드라이버 로직 추가 및 기능 확장 |
