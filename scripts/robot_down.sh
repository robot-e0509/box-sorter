#!/usr/bin/env bash
# robot_up.sh 로 띄운 로봇 드라이버와 그리퍼 서버를 내립니다.
#
#   ./scripts/robot_down.sh
#
# ros2 launch 는 자식 프로세스를 여러 개 띄우므로, 프로세스 그룹 전체를 종료합니다.
# (그래서 robot_up.sh 가 setsid 로 띄웁니다)
set -o pipefail

RUN_DIR="/tmp/doosan_ws_run"

ok()  { printf '  \033[1;32m✓\033[0m %s\n' "$*"; }
inf() { printf '  \033[0;90m·\033[0m %s\n' "$*"; }

stop() {
    local name="$1" pidfile="$RUN_DIR/$2.pid"

    if [[ ! -f "$pidfile" ]]; then
        inf "$name — 실행 기록 없음"
        return
    fi

    local pid
    pid=$(cat "$pidfile")

    if ! kill -0 "$pid" 2>/dev/null; then
        inf "$name — 이미 종료됨"
        rm -f "$pidfile"
        return
    fi

    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null

    for i in $(seq 1 10); do              # 최대 5초 기다림
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.5
    done

    if kill -0 "$pid" 2>/dev/null; then   # 안 죽으면 강제 종료
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
        ok "$name 강제 종료 (PID $pid)"
    else
        ok "$name 종료 (PID $pid)"
    fi
    rm -f "$pidfile"
}

printf '\n\033[1;36m▸ 로봇 서버 종료\033[0m\n'
stop "그리퍼 서버"   gripper
stop "로봇 드라이버" bringup

# 스크립트를 안 거치고 직접 띄운 것들도 정리.
#
# ※ pkill -f 를 그냥 쓰면 안 됩니다. 이 스크립트를 호출한 셸의 커맨드라인에
#   검색어가 들어 있으면(예: 다른 스크립트가 이 파일명을 인자로 넘긴 경우)
#   자기 자신까지 죽습니다. 그래서 자기 자신과 조상 프로세스를 제외합니다.
PATTERN="dsr_bringup2|gripper_service|ros2_control_node"

# 나 자신과 내 조상들의 PID 를 모은다
SELF=()
p=$$
while [[ -n "$p" && "$p" != "1" ]]; do
    SELF+=("$p")
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
done

LEFTOVER=()
while read -r pid _; do
    [[ -z "$pid" ]] && continue
    for s in "${SELF[@]}"; do [[ "$pid" == "$s" ]] && continue 2; done
    LEFTOVER+=("$pid")
done < <(pgrep -af "$PATTERN" 2>/dev/null || true)

if (( ${#LEFTOVER[@]} > 0 )); then
    printf '\n  \033[0;33m남아있는 프로세스를 정리합니다:\033[0m\n'
    for pid in "${LEFTOVER[@]}"; do
        printf '    %s  %.60s\n' "$pid" "$(ps -o args= -p "$pid" 2>/dev/null)"
    done
    kill -TERM "${LEFTOVER[@]}" 2>/dev/null
    sleep 2
    kill -KILL "${LEFTOVER[@]}" 2>/dev/null
    ok "정리 완료"
fi

printf '\n  로그는 남아 있습니다 → %s\n\n' "$RUN_DIR"
