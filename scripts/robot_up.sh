#!/usr/bin/env bash
# 로봇 드라이버 + 그리퍼 서버를 한 번에 띄우고 연결을 확인합니다.
#
#   ./scripts/robot_up.sh              # 기본 (RViz 포함)
#   ./scripts/robot_up.sh --no-rviz    # RViz 없이 (가벼움)
#   ROBOT_IP=110.120.1.99 ./scripts/robot_up.sh    # IP 바꿔서
#
# 끄려면:  ./scripts/robot_down.sh
set -o pipefail   # ※ set -u 는 쓰지 마세요. ROS 의 setup.bash 가 미정의 변수를
                  #   참조해서 "AMENT_TRACE_SETUP_FILES: unbound variable" 로 죽습니다.

ROBOT_IP="${ROBOT_IP:-110.120.1.68}"
MODEL="${MODEL:-e0509}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="/tmp/doosan_ws_run"

# doosan-robot2 에는 rviz 없는 launch 파일이 따로 없습니다.
# 대신 같은 launch 파일의 gui 인자로 RViz 를 끕니다.
GUI="true"
[[ "${1:-}" == "--no-rviz" ]] && GUI="false"

mkdir -p "$RUN_DIR"

say()  { printf '\n\033[1;36m▸ %s\033[0m\n' "$*"; }
ok()   { printf '  \033[1;32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[1;31m✗\033[0m %s\n' "$*"; }
die()  { bad "$*"; echo; exit 1; }

# ── 0. 이미 떠 있나 ──────────────────────────────────
if [[ -f "$RUN_DIR/bringup.pid" ]] && kill -0 "$(cat "$RUN_DIR/bringup.pid")" 2>/dev/null; then
    die "이미 실행 중입니다. 먼저 ./scripts/robot_down.sh 로 내려주세요."
fi

# ── 1. 워크스페이스 ──────────────────────────────────
say "워크스페이스 source"
source /opt/ros/jazzy/setup.bash
[[ -f "$WS/install/setup.bash" ]] || die "install/ 이 없습니다. 먼저 colcon build 하세요."
source "$WS/install/setup.bash"
ok "ROS 2 $ROS_DISTRO · $WS"

# ── 2. 네트워크 ──────────────────────────────────────
say "로봇 연결 확인 ($ROBOT_IP)"
ping -c 2 -W 2 "$ROBOT_IP" >/dev/null 2>&1 \
    || die "ping 실패. 랜선이 공유기에 꽂혀 있는지, IP가 맞는지 확인하세요."
ok "ping 응답"

timeout 3 bash -c "cat < /dev/null > /dev/tcp/$ROBOT_IP/12345" 2>/dev/null \
    && ok "컨트롤러 포트(12345) 열림" \
    || bad "포트 12345 응답 없음 — 컨트롤러 전원을 확인하세요 (계속 시도합니다)"

# ── 3. 로봇 드라이버 ─────────────────────────────────
say "로봇 드라이버 기동"
setsid ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real host:="$ROBOT_IP" model:="$MODEL" gui:="$GUI" \
    > "$RUN_DIR/bringup.log" 2>&1 &
echo $! > "$RUN_DIR/bringup.pid"

for i in $(seq 1 45); do
    grep -q "INITIAL STATE_STANDBY" "$RUN_DIR/bringup.log" 2>/dev/null && break
    if grep -qiE "STATE_SAFE_STOP|STATE_EMERGENCY_STOP" "$RUN_DIR/bringup.log" 2>/dev/null; then
        die "로봇이 비상정지/안전정지 상태입니다. 티치펜던트에서 해제하세요."
    fi
    kill -0 "$(cat "$RUN_DIR/bringup.pid")" 2>/dev/null || die "드라이버가 죽었습니다 → $RUN_DIR/bringup.log"
    sleep 1
done
grep -q "INITIAL STATE_STANDBY" "$RUN_DIR/bringup.log" \
    || die "45초 안에 STANDBY 가 안 됐습니다. 로그를 보세요 → $RUN_DIR/bringup.log"
ok "DRCF 연결 · STATE_STANDBY"

# ── 4. 그리퍼 서버 ───────────────────────────────────
say "그리퍼 서버 기동"
setsid ros2 run dsr_gripper gripper_service > "$RUN_DIR/gripper.log" 2>&1 &
echo $! > "$RUN_DIR/gripper.pid"

for i in $(seq 1 20); do
    grep -q "그리퍼 서비스 준비됨" "$RUN_DIR/gripper.log" 2>/dev/null && break
    sleep 1
done
grep -q "그리퍼 서비스 준비됨" "$RUN_DIR/gripper.log" \
    || die "그리퍼 서버가 안 떴습니다 → $RUN_DIR/gripper.log"
ok "/dsr01/gripper/cmd 준비됨"

# ── 5. 상태 확인 ─────────────────────────────────────
say "로봇 상태"
MODE=$(timeout 10 ros2 service call /dsr01/dsr_controller2/system/get_robot_mode \
       dsr_msgs2/srv/GetRobotMode 2>/dev/null | grep -o 'robot_mode=[0-9]*' | cut -d= -f2)
if [[ "$MODE" == "1" ]]; then
    ok "AUTONOMOUS 모드 (그리퍼 제어 가능)"
else
    bad "모드가 AUTONOMOUS 가 아닙니다 (현재: ${MODE:-?}). 그리퍼가 안 움직일 수 있습니다."
fi

cat <<EOF

  준비 완료.

    그리퍼 테스트   ./scripts/gripper.sh open
                    ./scripts/gripper.sh close 300
    로그 보기       tail -f $RUN_DIR/bringup.log
                    tail -f $RUN_DIR/gripper.log
    종료            ./scripts/robot_down.sh

  ※ 다른 터미널에서 ros2 명령을 쓰려면 그 터미널에서도:
       cd ~/doosan_ws && source install/setup.bash

EOF
