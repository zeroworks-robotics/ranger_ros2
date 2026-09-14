# ranger_ros2 정밀 분석 리포트

> 분석 대상: `/home/cona/workspace/git/ranger_ros2` (branch: `humble`)
> 의존 SDK: `/home/cona/workspace/git/ugv_sdk`
> 작성일: 2026-06-25

---

## 1. 프로젝트 개요

AgileX/Weston Robot **Ranger 시리즈 모바일 로봇**의 ROS2 (Humble) 드라이버.
CAN 버스로 하드웨어와 통신하며 `cmd_vel` 명령을 받아 구동하고, 로봇 상태·오도메트리를 발행한다.
핵심 특징은 **4륜 독립 조향(4WS)** 기반의 4가지 주행 모드 지원이다.

### 패키지 구성 (colcon workspace 3개 패키지)

| 패키지 | 역할 | 빌드 타입 |
|--------|------|-----------|
| `ranger_base` | 핵심 드라이버 노드 (CAN 통신, 운동학, 오도메트리, cmd_vel 처리) | ament_cmake (C++14) |
| `ranger_msgs` | 커스텀 메시지 6종 정의 | ament_cmake + rosidl |
| `ranger_bringup` | 모델별 런치 파일 + CAN-USB 설정 스크립트 | ament_cmake (launch only) |

### 외부 의존성
- **ugv_sdk** — AgileX CAN 프로토콜 SDK (`RangerRobot`, `RangerActuatorState` 등 제공)
- **boost::numeric::odeint** — 오도메트리 ODE 적분 (Runge-Kutta4)
- **eigen3**, **tf2 / tf2_ros / tf2_geometry_msgs**
- 표준 메시지: nav_msgs, sensor_msgs, geometry_msgs, std_msgs

---

## 2. 아키텍처 & 데이터 흐름

```
                    ┌─────────────────────────────────────┐
 /cmd_vel (Twist) ─▶│         RangerROSMessenger          │
                    │                                     │
                    │  TwistCmdCallback()                 │──▶ ugv_sdk RangerRobot
                    │   ├ 주행모드 판별                    │     (CAN0, 500kbps)
                    │   └ SetMotionMode / SetMotionCommand │
                    │                                     │
   ranger_base_node │  Run() @ 50Hz                       │◀── GetRobotState()
   (main)           │   └ PublishStateToROS()             │    GetActuatorState()
                    │       ├ UpdateOdometry() (odeint)   │    GetCommonSensorState()
                    └─────────────────────────────────────┘
                              │
        ┌──────────┬──────────┼───────────┬──────────────┐
        ▼          ▼          ▼           ▼              ▼
  /system_state /motion_state /actuator_state /odom  /battery_state  (+ TF odom→base_link)
```

- **엔트리포인트**: `ranger_base_node.cpp::main()` → `RangerROSMessenger` 생성 → CAN 연결 + `EnableCommandedMode()` → `Run()`
- **메인 루프**: `Run()`이 `rclcpp::Rate(update_rate_)`로 50Hz 폴링, 매 사이클 `PublishStateToROS()` + `spin_some()`

### 발행 토픽
| 토픽 | 타입 |
|------|------|
| `/system_state` | ranger_msgs/SystemState |
| `/motion_state` | ranger_msgs/MotionState |
| `/actuator_state` | ranger_msgs/ActuatorStateArray |
| `/odom` (이름 파라미터화) | nav_msgs/Odometry |
| `/battery_state` | sensor_msgs/BatteryState |
| TF: `odom`→`base_link` | (publish_odom_tf=true 시) |

### 구독 토픽
- `/cmd_vel` (geometry_msgs/Twist)

### 파라미터
`port_name`(can0), `robot_model`(ranger/mini_v1/v2/v3), `odom_frame`, `base_frame`, `odom_topic_name`, `update_rate`(50), `publish_odom_tf`(false)

---

## 3. 핵심 로직: 4가지 주행 모드

`TwistCmdCallback`에서 Twist 메시지를 분석해 모드를 자동 전환한다.

| 모드 | 트리거 조건 | 운동학 모델 |
|------|-------------|-------------|
| **DUAL_ACKERMAN** (기본) | `linear.y==0` 이고 `radius >= min_turn_radius` | 전·후륜 역위상 조향, RK4 적분 |
| **PARALLEL** (게걸음) | `linear.y != 0` (V1은 linear.x≠0 동반) | 전 바퀴 동일각, 차체 회전 없음 |
| **SPINNING** (제자리 회전) | `linear.y==0` 이고 `radius < min_turn_radius` | θ만 변화 |
| **SIDE_SLIP** | V1 전용, `linear.x==0 && linear.y!=0` | phi=90° 고정 |

- 운동학 모델은 `kinematics_model.hpp`에 ODE functor로 정의 (`DualAckermanModel`, `ParallelModel`, `SpinningModel`).
- `UpdateOdometry()`에서 RK4(`integrate_const`, step=dt/10)로 적분해 (x, y, θ) 추정.
- 조향각 변환: `ConvertInnerAngleToCentral()` — 내측 바퀴각 ↔ 차체 중심각.

### 모델별 파라미터 (`ranger_params.hpp`)
| 모델 | track(m) | wheelbase(m) | max_lin(m/s) | max_ang(rad/s) | min_turn_radius |
|------|----------|--------------|--------------|----------------|-----------------|
| Ranger | 0.56 | 0.90 | 2.7 | 0.785 | 0.810 |
| Mini V1 | 0.36 | 0.36 | 1.5 | 0.3 | 0.536 |
| Mini V2 | 0.364 | 0.494 | 1.5 | 4.8 | 0.476 |
| Mini V3 | 0.364 | 0.494 | 1.5 | 4.8 | 0.476 |

---

## 4. 발견된 이슈 (우선순위순)

### 🔴 Critical

**C1. actuator_state 발행 — 8개 모터 전부 0번 데이터로 채워짐**
`ranger_messenger.cpp:235-259`. `for(i=0..7)` 루프에서 인덱스 `i`를 전혀 사용하지 않고 포인터 디레퍼런스(`actuator_ls_state->`, `actuator_hs_state->`)만 한다 → 항상 `[0]`번 모터값. 각도/속도도 `angle_5`, `speed_1`로 고정.
- SDK 구조 확인 결과: `RangerActuatorState`는 `actuator_hs_state[8]`, `actuator_ls_state[8]` 배열 + `motor_angles(angle_5~8)` + `motor_speeds(speed_1~4)`.
- 네이밍 분석상 **4륜 × (구동+조향) = 8모터**: `speed_1~4`→actuator 0~3(구동), `angle_5~8`→actuator 4~7(조향).
- **영향**: 나머지 7개 모터의 전압/온도/RPM/전류/각도/속도 정보가 전부 유실. 모니터링·진단 불가.
- **수정**: `[i]` 인덱싱 + speed/angle를 인덱스에 맞게 매핑.

### 🟠 High

**H1. update_rate 런치 인자가 노드에 전달되지 않음**
`ranger_bringup/launch/*.launch.py`에서 `update_rate_arg`를 `DeclareLaunchArgument` 및 `LaunchDescription` 목록에는 넣었으나, **노드 `parameters={}` dict에는 빠져 있음**. 결과적으로 런치에서 update_rate를 바꿔도 무시되고 항상 기본값 50Hz 사용.

**H2. Mini V1 액추에이터 데이터 미수신 가능성**
SDK는 V1 전용 `ActuatorStateMessageV1`(current/rpm/temp) 구조를 별도로 둠. 그러나 메신저 코드는 V2 프로토콜의 `actuator_hs_state/ls_state`만 읽음 → V1에서 actuator 토픽이 비거나 0일 수 있음. (실하드웨어 검증 필요)

**H3. SIGINT 처리 시 비정상 종료**
`ranger_base_node.cpp`의 `SignalHandler`가 `exit(EXIT_FAILURE)` 호출. `rclcpp::shutdown()` 미호출 → 퍼블리셔/CAN 소켓 정리 없이 강제 종료, 종료 코드도 실패(1). Ctrl+C 정상 종료 흐름이 깨짐.

### 🟡 Medium

**M1. 주행모드 전환 로직 중복/도달불가 코드**
`TwistCmdCallback`의 `case PARALLEL` 내부(431-467)에 `linear.x==0 && linear.y!=0` SIDE_SLIP 처리 블록이 있으나, 그 조건은 상위 분기(400행)에서 이미 SIDE_SLIP(V1) 또는 PARALLEL로 갈라져 도달 경로가 모호하다. `case SIDE_SLIP`(480)와 로직이 분산되어 가독성·유지보수성 저하.

**M2. 저속에서 의도치 않은 SPINNING 전환**
`CalculateSteeringAngle`에서 `radius = linear/angular`. 저속(작은 linear)에 작은 각속도만 줘도 radius가 `min_turn_radius` 아래로 떨어져 제자리회전 모드로 급전환될 수 있음 → 주행감 저하. (커밋 689a8f3에서 부분 개선됨)

**M3. CalculateSteeringAngle의 하드코딩 매직넘버**
521행 `max_phi_rad = 40.0 deg`가 `robot_params_.max_steer_angle_*`를 무시하고 모든 모델에 일괄 적용. 모델별 파라미터 체계와 불일치. 또한 517행 `x = sqrt(...)`는 계산 후 미사용(주석 처리된 식의 잔재).

### 🟢 Low / 정리 대상

- **L1. 사용되지 않는 파라미터**: `max_speed_cmd`, `max_steer_angle_central`, `max_round_angle` — `LoadParameters`에서 할당만 하고 실제 사용처 없음.
- **L2. 죽은 코드**: `PublishSimStateToROS()` 선언만 있고 정의 없음. `ConvertCentralAngleToInner()` 정의됐으나 호출처 없음.
- **L3. simulated_robot 파라미터**: 런치에서 전달하지만 `LoadParameters`가 declare/read하지 않음 → 무의미.
- **L4. 런치 파일 중복**: `ranger_base/launch/`와 `ranger_bringup/launch/`에 거의 동일한 런치 파일 4쌍이 중복 존재. 유지보수 시 불일치 위험.
- **L5. ranger_mini_v1 bringup 런치**: `use_sim_time` 관련 라인이 전부 주석 처리됨(다른 모델과 비대칭).
- **L6. 파라미터 주석 오류**: `max_steer_angle_parallel = 1.570`에 "40 degree"(Ranger)/"180 degree"(V2/V3) 주석 — 실제 1.570rad ≈ 90°. 주석이 값과 불일치.
- **L7. package.xml 메타데이터**: author/maintainer가 `TODO`. `controller_manager` 의존성 선언됐으나 코드·CMake에서 미사용.
- **L8. BatteryState.present**: `quiet_NaN<uint8_t>()`는 uint8에 의미 없음(0이 됨). `power_supply_status` 등 항상 UNKNOWN 고정.
- **L9. README의 Services 섹션 비어 있음** — 실제 서비스 구현 없음(문서/코드 불일치).
- **L10. 매직넘버 `for(i=0;i<8;i++)`** — 모터 수 하드코딩.

---

## 5. 코드 품질 총평

| 항목 | 평가 |
|------|------|
| 구조/모듈화 | 양호 — 메신저/노드/파라미터/운동학 분리 명확 |
| 운동학 구현 | 양호 — odeint 기반 모드별 모델 깔끔 |
| 하드웨어 추상화 | 양호 — ugv_sdk로 CAN 세부 은닉 |
| 상태 발행 정확성 | **취약** — C1(actuator 버그)로 핵심 진단데이터 유실 |
| 런치/설정 일관성 | 보통 — H1, L4, L5 등 배선·중복 문제 |
| 에러/종료 처리 | **취약** — H3, CAN 연결 실패 시 노드는 살아있으나 무동작 |
| 죽은 코드/정리 | 보통 — L1~L3, L7 정리 여지 |

### 우선 권장 조치
1. **C1 수정** (actuator 인덱싱) — 즉시, 영향 큼
2. **H1 수정** (update_rate 배선) — 런치 dict에 한 줄 추가
3. **H3 개선** (정상 종료) — `rclcpp::shutdown()` 경유 graceful shutdown
4. M1/M3 리팩토링, L1~L3 죽은 코드 제거 — 후속 정리

---

## 6. 부록: 파일 인덱스

```
ranger_base/
  src/ranger_base_node.cpp        # main, SIGINT 핸들러
  src/ranger_messenger.cpp        # 핵심: cmd_vel 처리, 상태/오도메트리 발행
  include/ranger_base/
    ranger_messenger.hpp          # 클래스 선언
    ranger_params.hpp             # 모델별 물리 파라미터
    kinematics_model.hpp          # ODE 운동학 모델 3종
  launch/                         # (중복) 런치 4종
ranger_msgs/msg/                  # SystemState, MotionState, ActuatorState(Array),
                                  #  DriverState, MotorState
ranger_bringup/
  launch/                         # 실사용 진입 런치 4종 (ranger, mini_v1/v2/v3)
  scripts/                        # setup_can2usb.bash, bringup_can2usb.bash
```
