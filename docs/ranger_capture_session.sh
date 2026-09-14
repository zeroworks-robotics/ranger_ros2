#!/usr/bin/env bash
# 실기 세션용 캡처 스크립트 — CAN raw 와 ROS 토픽을 같은 타임라인에 동시 녹화한다.
#
#   사용법:  ./ranger_capture_session.sh record [초] [CAN인터페이스]
#            ./ranger_capture_session.sh 1g     [반복] [CAN인터페이스]
#
#   record : candump 와 ros2 bag 을 동시에 돌린다. 둘을 사후에 대조하지 않고
#            같은 주행에서 직접 비교하기 위한 것이다. 지금까지 CAN 경로 값과
#            ROS 경로 값이 갈릴 때마다 재표본화 실험으로 추론해야 했다.
#   1g     : 모드 전환 지연 측정. 0x141(SPINNING 명령) 송신 시각과, 그 뒤
#            0x281(휠속도)이 임계를 넘는 시각의 차. 0x281 은 mm/s 라 "바이트가
#            0이 아니면 움직였다"로 재면 양자화 바닥(1 mm/s)을 재게 되므로,
#            비영 / >0.02 / >0.05 / >0.10 m/s 네 기준을 한 번에 찍는다.
#            임계 없이는 정의되지 않는 양이다 — 결과를 옮길 때 임계를 같이 적을 것.
#            이 측정이 재는 것은 "구동 게이팅 해제" 시점이다(시뮬레이터 약 0.32초).
#            조향 완주 0.35초, 0x291 전환 플래그 0.57초는 각각 다른 시점이다.
#
#   주의: 1g 는 로봇이 제자리에서 돕니다. 회차마다 모드를 되돌리려고 6 cm 씩
#         전진하므로 ${N}회면 그만큼 앞으로 갑니다. 바퀴를 띄우거나 2m 이상 확보하세요.
#         리모컨은 CAN 모드로 두고, /cmd_vel 발행자가 하나인지 확인하세요.
set -u
MODE="${1:-}"
IF="${3:-can0}"
OUT="${RANGER_CAPTURE_DIR:-$HOME/ranger_capture}"

die() { echo "  $1" >&2; exit 1; }

check_cmd_vel_owner() {
  local n
  n=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/{print $3}')
  if [ "${n:-0}" -gt 0 ]; then
    echo "  !! /cmd_vel 을 이미 발행하는 노드가 ${n} 개 있습니다."
    echo "     명령이 번갈아 들어가면 결과가 실행마다 달라집니다. 멈추고 다시 실행하세요."
    # 대화형이 아니면(자동화·원격 실행) 묻지 않고 멈춘다. 조용히 기다리다
    # 세션이 끊기는 것보다, 중단하고 이유를 남기는 쪽이 낫다.
    if [ ! -t 0 ]; then
      echo "     대화형 터미널이 아니므로 중단합니다. 발행자를 멈추고 다시 실행하세요."
      exit 1
    fi
    read -r -p "     그래도 진행할까요? [y/N] " a
    [ "$a" = "y" ] || exit 1
  fi
}

case "$MODE" in
record)
  SEC="${2:-120}"
  command -v candump >/dev/null || die "candump 이 없습니다 (can-utils 설치 필요)"
  ip link show "$IF" >/dev/null 2>&1 || die "$IF 가 없습니다"
  mkdir -p "$OUT"
  STAMP=$(date +%Y%m%d_%H%M%S)
  D="$OUT/$STAMP"; mkdir -p "$D"
  echo "=== 동시 녹화 ${SEC}초 → $D"
  echo "    CAN: $IF (candump -ta, 전체 ID)"
  echo "    ROS: /system_state /motion_state /actuator_state /odom /bms_state /battery_state /cmd_vel"
  # -ta = 절대 시각. ros2 bag 도 같은 system clock 을 쓰므로 두 기록이 같은 타임라인에 놓인다.
  candump -ta "$IF" > "$D/can_$STAMP.log" &
  CAN_PID=$!
  ros2 bag record -o "$D/bag_$STAMP" \
    /system_state /motion_state /actuator_state /odom /bms_state /battery_state /cmd_vel \
    > "$D/bag.log" 2>&1 &
  BAG_PID=$!
  trap 'kill -INT "$BAG_PID" 2>/dev/null; kill -INT "$CAN_PID" 2>/dev/null' INT TERM
  # 로봇을 주행시키는 것은 사람이 한다 — 이 스크립트는 기록만 한다.
  echo "    지금 로봇을 평소대로 운전하세요. 끝나면 자동으로 멈춥니다 (Ctrl-C 도 가능)."
  sleep "$SEC"
  kill -INT "$BAG_PID" 2>/dev/null; kill -INT "$CAN_PID" 2>/dev/null
  wait 2>/dev/null
  echo
  echo "    CAN 프레임 $(wc -l < "$D/can_$STAMP.log") 줄"
  du -sh "$D" | awk '{print "    합계 " $1}'
  echo "    두 기록의 시각이 같은 clock 이어야 대조가 됩니다. ros2 bag 의 use_sim_time 이 꺼져 있는지 확인하세요."
  ;;
1g)
  N="${2:-5}"
  command -v candump >/dev/null || die "candump 이 없습니다 (can-utils 설치 필요)"
  check_cmd_vel_owner
  mkdir -p "$OUT"
  echo "=== 1g 모드 전환 지연 — ${N}회"
  echo "    0x141(SPINNING 송신) → 0x281 휠속도가 각 임계를 넘는 시각"
  echo "    비영 기준은 1 mm/s 에서 참이 되므로 그것만 인용하지 마세요."
  for i in $(seq 1 "$N"); do
    LOG=$(mktemp "${TMPDIR:-/tmp}/ranger1g.XXXXXX")
    # 141 = 모드 명령, 281 = 휠속도 피드백. 둘만 받는다.
    candump -ta "$IF,141:7ff,281:7ff" > "$LOG" &
    CP=$!
    sleep 0.5
    # 먼저 DUAL_ACKERMAN 으로 내려놓는다. 정지 명령(전부 0)은 "현재 모드 유지"라
    # 직전 회차의 SPINNING 이 그대로 남는다 — 그러면 다음 전환이 안 일어나고
    # 0x141 도 안 나간다. 그래서 아주 느리게 전진시켜 모드를 바꿔 놓는다
    # (0.05 m/s × 1.2초 ≈ 6 cm). cmd_deadzone 기본값 0.01 보다 커야 has_x 가 선다.
    timeout 1.2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1
    # 0x111 수신 타임아웃(500 ms)으로 섀시가 스스로 멈추도록 둔다.
    sleep 0.8
    timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}' >/dev/null 2>&1
    sleep 0.3
    kill -INT "$CP" 2>/dev/null; wait "$CP" 2>/dev/null
    awk '
      # (1725...) can0 141 [8] 02 00 ...   /   (...) can0 281 [8] xx xx ...
      #
      # 0x281 은 모터 속도 4개를 int16 big-endian mm/s 로 싣는다. "바이트가 0이
      # 아니면 움직인 것"으로 판정하면 1 mm/s 에서 참이 되어, 주행 시작이 아니라
      # 양자화 시작점을 재게 된다(실캡처에서 그 기준의 중위가 0.017초였다).
      # 그래서 실제 속도로 디코드해 여러 임계에서 동시에 잰다. 임계 없이는
      # 이 양이 정의되지 않으므로 임계를 항상 같이 찍는다.
      function hx(c,   p) { p = index("0123456789abcdef", tolower(c)); return p - 1 }
      function byte(h)    { return hx(substr(h,1,1)) * 16 + hx(substr(h,2,2)) }
      function i16(hi,lo,   v) { v = byte(hi) * 256 + byte(lo); if (v > 32767) v -= 65536; return v }
      BEGIN { split("0.02 0.05 0.10", THR, " ") }
      {
        ts = $1; gsub(/[()]/, "", ts)
        id = $3
        if (id == "141" && $5 == "02") { t141 = ts; seen = 1; next }   # data[0]=02 = SPINNING
        if (id == "281" && seen) {
          mx = 0
          for (k = 0; k < 4; k++) {
            v = i16($(5 + 2*k), $(6 + 2*k)) / 1000.0     # mm/s -> m/s
            if (v < 0) v = -v
            if (v > mx) mx = v
          }
          if (mx > 0 && nz == "") nz = ts - t141
          for (k in THR) if (hit[k] == "" && mx > THR[k] + 0.0) hit[k] = ts - t141
        }
      }
      END {
        if (!seen) { print "  " run "회차: 0x141 SPINNING 송신이 안 잡혔습니다 (모드 전환이 없었음)"; exit }
        line = "  " run "회차:"
        line = line sprintf("  비영 %s", (nz == "") ? "-" : sprintf("%.3f", nz))
        for (k = 1; k <= 3; k++)
          line = line sprintf("   >%s m/s %s", THR[k], (hit[k] == "") ? "-" : sprintf("%.3f", hit[k]))
        print line
        if (hit[3] == "") print "        (상위 임계 미도달 — 회전이 덜 붙었거나 창이 짧습니다)"
      }
    ' run="$i" "$LOG"
    rm -f "$LOG"
    sleep 1
  done
  echo
  echo "    각 줄은 같은 전환을 네 기준으로 잰 것입니다. 하나만 인용하지 말고 임계를 같이 적으세요."
  echo "    실캡처(RC 주행)에서는 비영 0.017초 / >0.05 0.374초 / >0.1 0.892초로 임계에 지배됐습니다."
  echo "    이 스크립트는 20 Hz 계단 입력을 주므로 조작자 램프가 섞이지 않아 더 작게 나옵니다."
  echo "    시뮬레이터는 조향 정착에서 유도해 약 0.35초, BLF 관측(0x291 전환 플래그)은 최대 0.57초입니다."
  ;;
*)
  sed -n '2,17p' "$0"
  exit 1
  ;;
esac
