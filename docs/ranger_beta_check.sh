#!/usr/bin/env bash
# ranger_ros2 / ugv_sdk beta 검증 스크립트 (0.1.0~beta1 / 0.2.0~beta1)
#
#   사용법:  ./ranger_beta_check.sh <항목번호> [CAN인터페이스]
#   예:      ./ranger_beta_check.sh 1          (can0 기본)
#            ./ranger_beta_check.sh 1 vcan0    (책상 시뮬레이터)
#
#   0 설치확인 / 1 PARALLEL조향 / 2 속도클램프 / 3 발행게이트
#   4 카운터동결 / 5 고장필드 / 6 BMS / 7 회귀
#
#   주의: 1·2·7 은 로봇이 움직입니다. 바퀴를 띄우거나 3m 이상 확보하세요.
#        리모컨은 CAN 모드(SWB 상단)로 고정하세요.
set -u
IF="${2:-can0}"
PASS=0; FAIL=0
ok()   { echo -e "  \033[32mPASS\033[0m  $1"; PASS=$((PASS+1)); }
bad()  { echo -e "  \033[31mFAIL\033[0m  $1"; FAIL=$((FAIL+1)); }
info() { echo "        $1"; }
hdr()  { echo; echo "=== $1 ==="; }
ask()  { read -rp "  >>> $1 [Enter 계속 / Ctrl-C 중단] "; }

field() {  # field <topic> <--field 경로>
  timeout 5 ros2 topic echo "$1" --field "$2" --once 2>/dev/null | head -1
}
steer() {  # 조향 모터(id 4~7) 중 첫 번째 각도. --field 는 배열 인덱싱을 못해서 파싱한다.
  timeout 5 ros2 topic echo /actuator_state --once 2>/dev/null \
    | grep -oP 'motor_angles: \K[-0-9.e+]+' | sed -n 5p
}
# 명령을 유지한 채 스트림을 받아 "마지막(=정착) 조향각"을 돌려준다.
# --once 를 특정 시점에 한 번 찍으면 조향이 아직 출발 전이거나 이동 중일 수 있다.
# 실섀시에서 이 때문에 정상 동작이 0.006 같은 값으로 읽혀 FAIL 로 보였다.
steer_settled() {  # steer_settled <스트림 초>
  timeout "$1" ros2 topic echo /actuator_state 2>/dev/null \
    | grep --line-buffered -oP 'motor_angles: \K[-0-9.e+]+' \
    | awk 'NR%8==5 {v=$0} END{print v}'
}
steer_all() {  # 조향 4축 전부 (id 4~7)
  timeout 5 ros2 topic echo /actuator_state --once 2>/dev/null \
    | grep -oP 'motor_angles: \K[-0-9.e+]+' | sed -n '5,8p' | tr '\n' ' '
}
pub() {    # pub <초> <twist yaml>
  timeout "$1" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "$2" >/dev/null 2>&1
}
stop() { timeout 1 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; sleep 1; }
# 명령을 "주는 도중에" 측정한다. 명령이 끊기면 섀시는 0x111 수신 타임아웃(500ms)으로
# 정지하고 바퀴도 복귀하므로, 끝난 뒤에 읽으면 전부 0 으로 보인다.
drive() {  # drive <초> <twist yaml> <측정명령...>
  local dur="$1" cmd="$2"; shift 2
  ( timeout "$dur" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "$cmd" >/dev/null 2>&1 ) &
  local pid=$!
  sleep 2
  "$@"
  wait $pid 2>/dev/null
}
near() {   # near <실측> <기대> <허용오차>
  awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<=t)}'
}

case "${1:-}" in
0)
  hdr "0. 설치 버전 확인"
  for p in ugv-sdk ranger-msgs ranger-base ranger-bringup; do
    v=$(apt-cache policy "zeroworks-humble-$p" 2>/dev/null | awk '/Installed/{print $2}')
    case "$v" in
      *~beta1*) ok "zeroworks-humble-$p = $v" ;;
      "")       bad "zeroworks-humble-$p 미설치" ;;
      *)        bad "zeroworks-humble-$p = $v  (기대: ~beta1)" ;;
    esac
  done
  ;;
1)
  hdr "1. PARALLEL 조향 — linear.y 는 정규화 조향 [-1,1] (최대 1.570 rad)"
  cm=$(field /system_state control_mode)
  [ "$cm" = "1" ] && ok "control_mode=1 (CAN)" || { bad "control_mode=$cm — 리모컨을 CAN 모드로 두세요"; exit 1; }
  ask "로봇이 움직입니다. 공간 확보됐습니까?"
  for c in "0.2 1.0 1.570" "0.2 -0.5 -0.785" "0.0 1.0 1.570"; do
    set -- $c; x=$1; y=$2; exp=$3
    stop
    ( timeout 9 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $x, y: $y}}" >/dev/null 2>&1 ) &
    pid=$!; ang=$(steer_settled 8); mm=$(field /motion_state motion_mode); wait $pid 2>/dev/null
    if near "$ang" "$exp" 0.08; then ok "x=$x y=$y → 조향 $ang (기대 $exp, mode=$mm)"
    else bad "x=$x y=$y → 조향 $ang (기대 $exp, mode=$mm)"; fi
  done
  stop
  hdr "1b. x 의존성 — y 가 같으면 x 가 달라도 조향각이 같아야 함"
  for x in 0.1 0.3 0.5; do
    stop
    ( timeout 9 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $x, y: 0.3}}" >/dev/null 2>&1 ) &
    pid=$!; ang=$(steer_settled 8); wait $pid 2>/dev/null
    if near "$ang" 0.471 0.08; then ok "x=$x y=0.3 → 조향 $ang (기대 0.471)"
    else bad "x=$x y=0.3 → 조향 $ang (기대 0.471)  ※ x 에 따라 변하면 옛 의미"; fi
  done
  stop
  ;;
2)
  hdr "2. 속도 클램프 — 0x111 linear 이 ±2000 mm/s 를 넘지 않아야 함"
  ask "과대 명령을 넣습니다. 바퀴를 띄웠거나 공간이 충분합니까?"
  for c in "5.0 0.0 2000" "40.0 0.0 2000" "2.0 3.0 1000"; do
    set -- $c; x=$1; wz=$2; lim=$3
    stop
    ( pub 4 "{linear: {x: $x}, angular: {z: $wz}}" ) &
    sleep 1
    vals=$(timeout 2 candump "$IF,111:7ff" 2>/dev/null | awk '{print $4$5}' | sort -u | head -5)
    wait
    worst=0
    for v in $vals; do
      d=$(( 16#$v )); [ $d -gt 32767 ] && d=$((d-65536))
      a=${d#-}; [ "$a" -gt "$worst" ] && worst=$a
      [ $d -lt 0 ] && neg=1
    done
    if [ "$worst" -le "$lim" ]; then ok "vx=$x wz=$wz → 최대 ${worst} mm/s (상한 $lim)"
    else bad "vx=$x wz=$wz → 최대 ${worst} mm/s (상한 $lim 초과)"; fi
  done
  stop
  ;;
3)
  hdr "3. 발행 게이트 — CAN 프레임이 없으면 아무것도 발행하지 않아야 함"
  info "섀시 전원을 끄거나 CAN 케이블을 뽑고, 노드를 '재시작'한 뒤 계속하세요."
  ask "노드가 프레임 없는 상태로 새로 떠 있습니까?"
  for t in /system_state /motion_state /battery_state /actuator_state /odom; do
    n=$(timeout 3 ros2 topic echo "$t" --once 2>/dev/null | wc -l)
    [ "$n" -eq 0 ] && ok "$t 미발행" || bad "$t 가 ${n}줄 발행됨"
  done
  info "노드 로그에 'No CAN feedback received yet on' 이 보이는지 확인하세요."
  ask "이제 섀시 전원을 켜거나 케이블을 다시 연결하세요."
  sleep 5
  v=$(field /system_state battery_voltage)
  [ -n "$v" ] && ok "복구 후 발행 재개 (battery_voltage=$v)" || bad "복구 후에도 발행 없음"
  ;;
4)
  hdr "4. 카운터 동결 — CAN 이 끊기면 메시지는 계속 나오고 카운터만 멈춰야 함"
  s1=$(field /system_state feedback_count); a1=$(field /actuator_state feedback_count); b1=$(field /bms_state feedback_count)
  sleep 4
  s2=$(field /system_state feedback_count); a2=$(field /actuator_state feedback_count); b2=$(field /bms_state feedback_count)
  [ "$s2" -gt "$s1" ] && ok "system  $s1 → $s2 (증가)" || bad "system  $s1 → $s2 (증가해야 함)"
  [ "$a2" -gt "$a1" ] && ok "actuator $a1 → $a2 (증가)" || bad "actuator $a1 → $a2 (증가해야 함)"
  [ "$b2" -gt "$b1" ] && ok "bms     $b1 → $b2 (증가)" || bad "bms     $b1 → $b2 (증가해야 함)"
  ask "이제 CAN 케이블을 뽑으세요 (노드는 그대로 두세요)."
  sleep 3
  s1=$(field /system_state feedback_count); a1=$(field /actuator_state feedback_count); b1=$(field /bms_state feedback_count)
  sleep 4
  s2=$(field /system_state feedback_count); a2=$(field /actuator_state feedback_count); b2=$(field /bms_state feedback_count)
  if [ -z "$s2" ]; then bad "끊긴 뒤 /system_state 가 아예 안 나옴 (마지막 값 유지가 기대 동작)"; else
    [ "$s2" -eq "$s1" ] && ok "system  동결 ($s2)" || bad "system  $s1 → $s2 (동결해야 함)"
    [ "$a2" -eq "$a1" ] && ok "actuator 동결 ($a2)" || bad "actuator $a1 → $a2 (동결해야 함)"
    [ "$b2" -eq "$b1" ] && ok "bms     동결 ($b2)" || bad "bms     $b1 → $b2 (동결해야 함)"
  fi
  ask "케이블을 다시 연결하세요."
  ;;
5)
  hdr "5. 고장 필드 — error_code_full 이 0x211 byte[4..7] 전체를 담아야 함"
  ef=$(field /system_state error_code_full); e=$(field /system_state error_code); vs=$(field /system_state vehicle_state)
  printf "        평상시: error_code=%s error_code_full=0x%08X vehicle_state=%s\n" "$e" "$ef" "$vs"
  info "0x100=드라이버 이상  0x80=비상정지  0x4=RC 연결상실  0x10000=조향 캘리브레이션"
  ask "비상정지 버튼을 누르세요."
  sleep 2
  ef=$(field /system_state error_code_full); vs=$(field /system_state vehicle_state)
  printf "        누른 뒤: error_code_full=0x%08X vehicle_state=%s\n" "$ef" "$vs"
  if [ $(( ef & 0x80 )) -ne 0 ]; then ok "비상정지 비트(0x80) 관측됨"; else bad "비상정지 비트가 보이지 않음 (0x%08X)"; fi
  [ "$vs" = "2" ] && ok "vehicle_state=2 (abnormal)" || info "vehicle_state=$vs"
  ask "비상정지를 풀고 리모컨 KEY1 로 에러를 지우세요 (0x441 미구현이라 ROS 로는 해제 불가)."
  ;;
6)
  hdr "6. /bms_state"
  for f in battery_soc battery_soh voltage current temperature feedback_count; do
    printf "        %-16s %s\n" "$f" "$(field /bms_state $f)"
  done
  bv=$(field /battery_state percentage); bs=$(field /bms_state battery_soc)
  near "$bv" "$bs" 0.5 && ok "/battery_state.percentage($bv) == /bms_state.battery_soc($bs)" \
                       || bad "/battery_state.percentage($bv) != /bms_state.battery_soc($bs)"
  ;;
7)
  hdr "7. 회귀 — 기존 동작"
  ask "로봇이 움직입니다. 공간 확보됐습니까?"
  stop
  vx=$(drive 6 "{linear: {x: 0.15}}" field /odom twist.twist.linear.x)
  printf "        ackerman  vx=%s mode=%s\n" "$vx" "$(field /motion_state motion_mode)"
  stop
  vx=$(drive 6 "{linear: {x: 0.15}, angular: {z: 0.3}}" field /odom twist.twist.linear.x)
  printf "        arc       vx=%s mode=%s\n" "$vx" "$(field /motion_state motion_mode)"
  stop
  wz=$(drive 6 "{angular: {z: 0.4}}" field /odom twist.twist.angular.z)
  printf "        spinning  wz=%s mode=%s (기대 mode=2)\n" "$wz" "$(field /motion_state motion_mode)"
  stop
  ros2 service call /set_parking_mode std_srvs/srv/SetBool "{data: true}" >/dev/null; sleep 2
  mm=$(field /motion_state motion_mode)
  [ "$mm" = "3" ] && ok "파킹 진입 (mode=3)" || bad "파킹 진입 실패 (mode=$mm)"
  vx=$(drive 5 "{linear: {x: 0.15}}" field /odom twist.twist.linear.x)
  near "$vx" 0 0.02 && ok "파킹 중 명령 무시 (vx=$vx)" || bad "파킹 중에 움직임 (vx=$vx)"
  ros2 service call /set_parking_mode std_srvs/srv/SetBool "{data: false}" >/dev/null; sleep 2
  mm=$(field /motion_state motion_mode)
  [ "$mm" != "3" ] && ok "파킹 해제 (mode=$mm)" || bad "파킹 해제 실패"
  stop
  ;;
1d)
  hdr "1d. 진단 — 조향각 시간 궤적"
  info "조향이 목표에 도달하는 데 걸리는 시간을 잽니다. 로봇이 10초씩 두 번 움직입니다."
  ask "공간 확보됐습니까?"
  for c in "0.2 1.0 1.570" "0.2 -0.5 -0.785"; do
    set -- $c; x=$1; y=$2; exp=$3
    stop
    echo "  --- x=$x y=$y (기대 $exp)"
    ( timeout 11 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $x, y: $y}}" >/dev/null 2>&1 ) &
    pid=$!
    # --once 를 반복하면 호출마다 2초씩 걸려 궤적이 뭉개진다. 스트림을 받아서
    # 조향 모터(5번째 motor_angles) 값만 뽑고 파이썬으로 경과시간을 붙인다.
    timeout 10 ros2 topic echo /actuator_state 2>/dev/null \
      | grep --line-buffered -oP 'motor_angles: \K[-0-9.e+]+' \
      | python3 -u -c "
import sys, time
t0 = time.time(); n = 0; last = None
for line in sys.stdin:
    n += 1
    if n % 8 != 5:      # 8축 중 5번째 = 조향 모터 첫 축
        continue
    v = float(line)
    t = time.time() - t0
    if last is None or abs(v - last) > 0.005 or int(t*2) % 4 == 0:
        print('      t=%5.1fs  조향=%+.3f' % (t, v)); last = v
" | awk 'NR<=25'
    wait $pid 2>/dev/null
  done
  stop
  info "목표 도달 시각을 보고 본 테스트의 샘플링 시점을 정합니다."
  ;;
*)
  echo "사용법: $0 <0-7|1d> [CAN인터페이스]"; echo "  0 설치 / 1 PARALLEL / 1d 조향궤적진단 / 2 클램프 / 3 게이트 / 4 카운터 / 5 고장 / 6 BMS / 7 회귀"; exit 1 ;;
esac
echo; echo "  합계: PASS $PASS / FAIL $FAIL"
[ "$FAIL" -eq 0 ] || exit 1
