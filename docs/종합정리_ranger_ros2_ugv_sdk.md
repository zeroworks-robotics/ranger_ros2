# Ranger 스택 종합 정리 — ranger_ros2 + ugv_sdk

> 통합 대상 문서
> - `ranger_ros2/docs/ranger_ros2_analysis.md` (ROS2 드라이버 정밀 분석, 2026-06-25)
> - `ugv_sdk/docs/CODEBASE_ANALYSIS.md` (SDK 코드베이스 분석, 2026-06-25)
> - `ranger_ros2/docs/작업메모_상태발행수정_2026-06-26.md` (PR #1)
> - `ranger_ros2/docs/작업메모_조향모드_2026-07-08.md` (PR #2)
>
> 정리일: 2026-07-08 · 대상 브랜치: `humble`

---

## 목차
1. [시스템 전체 구조](#1-시스템-전체-구조)
2. [ranger_ros2 (ROS2 드라이버 계층)](#2-ranger_ros2-ros2-드라이버-계층)
3. [ugv_sdk (SDK / CAN 프로토콜 계층)](#3-ugv_sdk-sdk--can-프로토콜-계층)
4. [수행한 작업 (PR #1, #2)](#4-수행한-작업-pr-1-2)
5. [조향 모드 상세 (twist vs direct)](#5-조향-모드-상세-twist-vs-direct)
6. [통합 이슈 트래커](#6-통합-이슈-트래커)
7. [운영 / 사용 가이드](#7-운영--사용-가이드)
8. [부록](#8-부록)

---

## 1. 시스템 전체 구조

```
 [ROS2 앱 / teleop / nav2]
        │  /cmd_vel (Twist)
        ▼
 ┌──────────────────────────┐   ranger_ros2 (본 리포)
 │ ranger_base_node         │   · cmd_vel → 주행모드 판별 → 명령
 │  └ RangerROSMessenger    │   · 50Hz 상태/오도메트리 발행
 └──────────────────────────┘
        │  RangerRobot API (SetMotionCommand / GetRobotState …)
        ▼
 ┌──────────────────────────┐   ugv_sdk (../ugv_sdk)
 │ AgilexBase<ParserType>   │   · AgxMessage ⇄ CAN frame 인코딩/디코딩
 │  └ RangerBase / V1·V2·V3 │   · ASIO 비동기 CAN RX 스레드
 └──────────────────────────┘
        │  SocketCAN (can0, 500kbps, 빅엔디안, /1000 SI)
        ▼
 [Ranger 펌웨어 / 4WS 하드웨어]
```

- **ranger_ros2**: ROS 인터페이스 + 4WS 운동학 + 오도메트리.
- **ugv_sdk**: CAN 프로토콜 구현(V1/V2), 로봇별 quirk 보정, 전송 계층.
- 두 리포는 동일 워크스페이스(`~/workspace/git`)에서 colcon으로 함께 빌드. 컨테이너 `cona_icn_ros2:0.0.1`(ROS2 Humble)에서 빌드/검증.

---

## 2. ranger_ros2 (ROS2 드라이버 계층)

### 2.1 패키지 구성
| 패키지 | 역할 | 빌드 |
|--------|------|------|
| `ranger_base` | 핵심 드라이버 노드 (CAN, 운동학, 오도메트리, cmd_vel) | ament_cmake (C++14) |
| `ranger_msgs` | 커스텀 메시지 6종 | ament_cmake + rosidl |
| `ranger_bringup` | 모델별 런치 + CAN-USB 스크립트 | ament_cmake |

**외부 의존성**: ugv_sdk, boost::odeint(RK4 오도메트리), eigen3, tf2 계열, nav_msgs/sensor_msgs.

### 2.2 데이터 흐름
- 엔트리: `ranger_base_node.cpp::main()` → `RangerROSMessenger` 생성 → CAN 연결 + `EnableCommandedMode()` → `Run()`.
- 메인 루프: `Run()`이 `update_rate_`(기본 50Hz)로 `PublishStateToROS()` + `spin_some()`.
- **발행**: `/system_state`, `/motion_state`, `/actuator_state`, `/odom`, `/battery_state`, (옵션) TF `odom→base_link`.
- **구독**: `/cmd_vel`.
- **파라미터**: `port_name`(can0), `robot_model`, `odom_frame`, `base_frame`, `odom_topic_name`, `update_rate`, `publish_odom_tf`, **`steer_mode`**(신규).

### 2.3 4가지 주행 모드
`TwistCmdCallback`이 Twist를 분석해 자동 전환:

| 모드 | 트리거 | 운동학 |
|------|--------|--------|
| **DUAL_ACKERMAN** (기본) | `linear.y==0`, 직진+선회 | 전·후륜 역위상 조향, RK4 |
| **PARALLEL** (게걸음) | `linear.y != 0` | 전 바퀴 동일각, 차체 회전 없음 |
| **SPINNING** (제자리 회전) | `linear.x≈0 && angular≠0` *(수정 후)* | θ만 변화 |
| **SIDE_SLIP** | V1 전용, `linear.x==0 && linear.y!=0` | phi=90° |

> ※ SPINNING 트리거는 원래 `radius < min_turn_radius` 였으나 PR #2에서 변경(→ [4.2](#42-pr-2--combined-명령--조향-모드-fixcombined-linear-angular-cmd)).

### 2.4 모델별 물리 파라미터 (`ranger_params.hpp`)
| 모델 | track(m) | wheelbase(m) | max_lin(m/s) | max_ang(rad/s) | min_turn_radius | max_steer_ackermann |
|------|----------|--------------|--------------|----------------|-----------------|---------------------|
| Ranger | 0.56 | 0.90 | 2.7 | 0.785 | 0.810 | 0.698 (40°) |
| Mini V1 | 0.36 | 0.36 | 1.5 | 0.3 | 0.536 | 0.698 |
| Mini V2 | 0.364 | 0.494 | 1.5 | 4.8 | 0.476 | 0.698 |
| Mini V3 | 0.364 | 0.494 | 1.5 | 4.8 | 0.476 | 0.698 |

---

## 3. ugv_sdk (SDK / CAN 프로토콜 계층)

> 버전: CMake `0.8.0` / package.xml `0.1.6` / CHANGELOG `0.1.5` (**3중 불일치**).
> 지원 로봇: Scout·Hunter·Bunker·Tracer·Ranger(Mini 1.0/2.0/3.0/표준)·Titan.

### 3.1 아키텍처 (4계층)
```
Public API   mobile_robot/*.hpp    런타임 enum → 템플릿 인스턴스 위임(어댑터)
Robot Base   details/robot_base/   AgilexBase<ParserType> + 로봇별 Base, 상태 뮤텍스, RX 디스패치
Protocol     details/protocol_v1|v2  ParserBase<Version>, CAN frame ⇄ AgxMessage
Transport    details/async_port/   AsyncCAN(SocketCAN)/AsyncSerial(UART), ASIO 이벤트 루프
```
설계 핵심: 컴파일타임 정책 주입(제로 오버헤드) + 런타임 어댑터 + 콜백 기반 RX + 펌웨어 quirk를 서브클래스 오버라이드로 패치 + 상태 3분할(core/actuator/common-sensor 별도 뮤텍스).

### 3.2 CAN 프로토콜 (V2 기준, 검증됨)
| 메시지 | V2 ID | 방향 | 비고 |
|--------|-------|------|------|
| Motion Command | 0x111 | TX | linear/angular/lateral/steering, /1000 SI, 빅엔디안 |
| System State | 0x211 | RX | vehicle_state, control_mode, battery_voltage |
| Motion State | 0x221 | RX | linear/angular/lateral vel, steering angle |
| Actuator HS | 0x251–258 | RX | rpm, current×0.1A, pulse_count |
| Actuator LS | 0x261–268 | RX | driver_voltage×0.1V, temp |
| Steering angle(5–8) | 0x271 | RX | 0.001 rad |
| Wheel speed(1–4) | 0x281 | RX | mm/s |
| BMS Basic | 0x361 | RX | **voltage 0.01V** (V3 미보정 주의) |
| Set motion mode | 0x141 | TX | 0=Ackerman,1=oblique,2=spin,3=park |

- **모터 번호**: 1=우전 / 2=우후 / 3=좌후 / 4=좌전(구동), 5~8=조향. `motor_id = can_id − 0x251`.
- **V1 vs V2**: V1은 퍼센트 int8·체크섬 검증·4모터 단일 프레임 / V2는 SI 단위·체크섬 미검증·8모터 HS+LS 분리.

### 3.3 SDK 발견 이슈
| # | 심각도 | 항목 | 상태 |
|---|--------|------|------|
| 0 | 🔴 | **Ranger Mini 3.0 배터리 전압 10배 과대 보고** — V3가 `RangerBase` alias라 0.01V 보정 누락(480V로 보고) | ✅ 수정 (V2 상속으로) |
| A | 🔴 | 버전 요청을 응답 ID(0x4a1)로 전송 → `RequestVersion()` 타임아웃 의심 | ⏳ 실기 검증 후 0x411 |
| B | 🟡 | LS 디코드가 union HS 멤버에 기록(우연히 동작) | ✅ 수정 |
| C-2 | 🔴 | Bunker V1 `actuator_state[2]` 배열에 `i<3` 쓰기 → 오버플로우 | ⏳ ranger 범위 밖 |
| D | 🟡 | V1 경로에서 angular/steering 중 절댓값 큰 쪽만 채택 | 전제상 정상 |

> SDK 측 0·B·회귀테스트는 브랜치 `fix/ranger-mini-v3-bms-voltage`에서 완료(컨테이너 빌드·utest PASS). A·C-2·버전메타·BMS 인코더는 후속.

---

## 4. 수행한 작업 (PR #1, #2)

### 4.1 PR #1 — 상태 발행 / 런치 / 종료 (`fix/state-publishing-and-shutdown`, 머지됨)
| 코드 | 항목 | 내용 |
|------|------|------|
| **C1** 🔴 | actuator 발행 버그 | 8모터 루프가 인덱스 없이 항상 `[0]`만 읽던 것을 `[i]` 인덱싱으로 수정. `speed_1~4`→id 0~3(구동), `angle_5~8`→id 4~7(조향) 매핑 |
| **H1** 🟠 | 런치 파라미터 배선 | 4개 런치에 `update_rate`·`robot_model` 누락 보강 (mini_v1/v2는 robot_model 미전달로 기본 `ranger` 파라미터 로드되던 버그 동반) |
| **H3** 🟠 | 정상 종료 | SIGINT `exit(EXIT_FAILURE)` → `rclcpp::shutdown()` |
| **M1/M3** 🟡 | 정리 (동작 불변) | `CalculateSteeringAngle` 매직넘버 40°→`max_steer_angle_ackermann`, 미사용 변수·죽은 주석 제거, `steer_cmd`/`radius` 초기화 |

**C1 매핑 근거**: Ranger는 4륜 × (구동+조향) = 8모터. SDK `RangerActuatorState`의 `actuator_hs/ls_state[8]` 배열 + `speed_1~4`/`angle_5~8` 네이밍, 파서 `motor_id = id−0x251`(0~7)로 교차 확인.

### 4.2 PR #2 — combined 명령 & 조향 모드 (`fix/combined-linear-angular-cmd`, 머지됨)
| 커밋 | 항목 | 내용 |
|------|------|------|
| `0fab72b` | combined 명령 버그 | linear+angular 동시 명령이 스핀으로 빠져 직진 성분이 사라지던 것 수정 |
| `495f588` | direct 조향 모드 | `steer_mode` 파라미터 추가 (`twist`/`direct`) |

**combined 버그 원인·수정**: 회전반경 `radius=|linear.x|/|angular.z|`이 `min_turn_radius` 미만이면 무조건 SPINNING → `SetMotionCommand(0,0,angular)`로 linear 버림. teleop 기본값(0.5 m/s, 1.0 rad/s = 반경 0.5m)이 항상 스핀으로 빠짐(조이스틱은 반경이 커서 정상).
```
before: radius < min_turn_radius              → SPINNING
after : |linear.x|<1e-6 && |angular.z|>1e-6   → SPINNING (순수 제자리회전만)
        그 외                                  → DUAL_ACKERMAN (조향 최대 클램프 → arc)
```

> **teleop 키 주의(버그 아님)**: 터미널은 한 번에 한 키만 읽어 각 키가 twist를 덮어씀. `i` 후 `l`은 교체(동시 아님). **동시 명령은 대각선 키 `u` `o` `m` `.` 로만** 나옴.

---

## 5. 조향 모드 상세 (twist vs direct)

`steer_mode` 파라미터로 선택. 기본 `twist`라 기존 동작 불변.

| | **twist** (기본) | **direct** |
|---|---|---|
| `angular.z` 의미 | 각속도(yaw rate, rad/s) | 조향각(rad) 직접 |
| 조향각 결정 | linear/angular 비율로 **계산** | angular.z **그대로** (max 클램프) |
| 속도 영향 | 있음 (빠르면 완만, 느리면 급) | **없음** (속도 무관, 핸들 각 고정) |
| `j`/`l` (회전만) | 제자리 회전(SPINNING) | 바퀴만 꺾이고 안 움직임 |
| 용도 | **자율주행/알고리즘**(nav2 등) | **조이스틱/수동 조종** (RC 느낌) |

**숫자 예시** (angular.z=0.5 고정):
- twist: 0.5 m/s→조향 ≈24°, 1.0 m/s→≈13° (속도 따라 자동 조절, 원 궤적 유지)
- direct: 0.5 m/s→28.6°, 1.0 m/s→28.6° (동일, RC 핸들과 같음)

### ⚠️ direct 모드 주의사항
1. **자율주행(nav2 등)과 호환 안 됨** — 가장 중요. angular.z를 조향각으로 해석하므로 yaw rate를 보내는 알고리즘과 함께 쓰면 오동작. **자율주행은 반드시 `twist`.**
2. **제자리 회전 불가** — `j`/`l`(회전만)은 바퀴만 꺾이고 안 움직임. 제자리 회전 필요 시 `twist`.
3. **조향각 상한 0.698 rad(40°), 초과분 클램프** — teleop 기본 turn=1.0은 항상 풀락. 중간 각도는 `turn ≈ 0.5`.
4. **조이스틱은 angular 스케일 조정 필요** — `teleop_twist_joy`의 `scale_angular ≈ 0.7`.
5. **속도 무관 = 고속에서 더 공격적** — 같은 조향각 유지 → 빠를수록 실제 선회 커짐.
6. **기본값은 twist라 옵트인** — `steer_mode:=direct` 명시할 때만 적용.

### direct 모드 조향각 세팅
| turn(angular.z) | 조향각 |
|---|---|
| 0.7 | 40° (풀락, 상한) |
| 0.5 | ~29° (권장) |
| 0.35 | ~20° |
| 0.17 | ~10° |

---

## 6. 통합 이슈 트래커

### ranger_ros2
| # | 심각도 | 항목 | 상태 |
|---|--------|------|------|
| C1 | 🔴 | actuator 발행(8모터 [0] 고정) | ✅ PR #1 |
| H1 | 🟠 | update_rate/robot_model 런치 미배선 | ✅ PR #1 |
| H3 | 🟠 | SIGINT 비정상 종료 | ✅ PR #1 |
| M1 | 🟡 | 주행모드 로직 정리 | ✅ PR #1 (부분) |
| M3 | 🟡 | 조향각 매직넘버 | ✅ PR #1 |
| — | 🔴 | combined 명령 스핀 전락 | ✅ PR #2 |
| H2 | 🟠 | **Mini V1 액추에이터 미수신 가능성** | ⏳ 실기 검증 필요 |
| M2 | 🟡 | 저속 의도치 않은 스핀 | 🔸 PR #2로 완화 |
| L1~L10 | 🟢 | 죽은 코드/주석/메타데이터/런치 중복 등 | ⏳ 선택 정리 |

### ugv_sdk
| # | 심각도 | 항목 | 상태 |
|---|--------|------|------|
| 0 | 🔴 | Mini 3.0 전압 10배 | ✅ (SDK 브랜치) |
| B | 🟡 | LS union 멤버 오기 | ✅ (SDK 브랜치) |
| A | 🔴 | 버전요청 ID 0x4a1 | ⏳ 실기 검증 |
| C-2 | 🔴 | Bunker V1 배열 오버플로우 | ⏳ 별도 |
| — | 🟢 | 버전 메타 3중 불일치 / BMS 인코더 미구현 / 테스트 사각지대 | ⏳ |

---

## 7. 운영 / 사용 가이드

### 실행
```bash
# 자율주행/알고리즘 (기본 twist)
ros2 launch ranger_bringup ranger.launch.py            # 또는 ranger_mini_v1/v2/v3

# 조이스틱/수동 (RC식 직접 조향)
ros2 launch ranger_bringup ranger.launch.py steer_mode:=direct
```

### CAN 세팅
```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000    # bringup_can2usb.bash
candump can0                                          # 수신 확인
```

### teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p turn:=0.5
# 동시 명령은 u/o/m/. 키. i/j/k/l/,는 단일 성분.
```

### 빌드 (컨테이너)
```bash
docker start docker_icn_ros2-cona_icn-1
docker exec docker_icn_ros2-cona_icn-1 bash -lc '
  source /opt/ros/humble/setup.bash && cd /root/git
  colcon build --packages-select ranger_msgs ugv_sdk ranger_base ranger_bringup'
```

### 진단 팁
```bash
ros2 topic echo /motion_state   # combined 명령 중 motion_mode:
                                 #   0 = DUAL_ACKERMAN (정상, 호 주행)
                                 #   2 = SPINNING (구버전 or 회전만)
ros2 topic echo /actuator_state  # 8모터 값이 서로 다른지 (C1 검증)
```

---

## 8. 부록

### 8.1 ranger_ros2 파일 인덱스
```
ranger_base/
  src/ranger_base_node.cpp        # main, SIGINT 핸들러
  src/ranger_messenger.cpp        # 핵심: cmd_vel 처리, 상태/오도메트리 발행
  include/ranger_base/
    ranger_messenger.hpp          # 클래스 선언 (direct_steer_ 멤버)
    ranger_params.hpp             # 모델별 물리 파라미터
    kinematics_model.hpp          # ODE 운동학 모델 3종
ranger_msgs/msg/                  # SystemState, MotionState, ActuatorState(Array), DriverState, MotorState
ranger_bringup/
  launch/                         # 실사용 런치 4종 (steer_mode 인자 포함)
  scripts/                        # setup_can2usb.bash, bringup_can2usb.bash
```

### 8.2 검증 환경
- 컨테이너 `cona_icn_ros2:0.0.1` (ROS2 Humble, `~/workspace/git` → `/root/git` 바인드 마운트)
- ranger_ros2: PR #1·#2 빌드 통과, twist·direct 실기 검증 완료
- 실기 미검증 잔여: H2(Mini V1 액추에이터), SDK-A(버전요청 ID)

### 8.3 참고 매뉴얼
- Ranger Mini 3.0 User Manual V1.0.0 (2024.06) §3.2 CAN Communication Protocol — SDK CAN ID/스케일 교차검증 근거.
