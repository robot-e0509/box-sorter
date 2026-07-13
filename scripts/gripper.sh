#!/usr/bin/env bash
# 그리퍼 제어 (robot_up.sh 로 서버가 떠 있어야 합니다)
#
#   ./scripts/gripper.sh open              # 열기
#   ./scripts/gripper.sh close             # 닫기 (기본 current 300)
#   ./scripts/gripper.sh close 250         # 닫기 (힘 250)
#   ./scripts/gripper.sh set 375 200       # 임의 위치(0~750) + 힘
#   ./scripts/gripper.sh sweep 150 400 50  # 파지 강도 실험:
#                                          #   150 → 400 까지 50 씩 올려가며 닫아본다
#
# position : 0 = 완전 닫힘 ~ 750 = 완전 열림
# current  : 파지 힘. 200~400 권장. 낮으면 미끄러지고 높으면 상자가 찌그러집니다.
set -o pipefail   # set -u 금지 — ROS setup.bash 가 미정의 변수를 참조합니다.

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

SRV=/dsr01/gripper/cmd

cmd() {   # cmd <position> <current>
    local pos="$1" cur="$2"
    printf '  pos=%-4s cur=%-4s ' "$pos" "$cur"
    if timeout 30 ros2 service call "$SRV" dsr_gripper_interfaces/srv/GripperCmd \
         "{position: $pos, current: $cur}" 2>/dev/null | grep -q "success=True"; then
        printf '\033[1;32m전송됨\033[0m\n'
    else
        printf '\033[1;31m실패\033[0m — 그리퍼 서버가 떠 있나요? (./scripts/robot_up.sh)\n'
        exit 1
    fi
}

case "${1:-}" in
    open)   cmd 750 "${2:-200}" ;;
    close)  cmd 0   "${2:-300}" ;;
    set)    cmd "${2:?position(0~750) 필요}" "${3:-200}" ;;

    sweep)
        # 파지 강도 실험 — 동적 강도 팀에 필요한 데이터를 만드는 명령입니다.
        lo="${2:-150}"; hi="${3:-400}"; step="${4:-50}"
        echo
        echo "  파지 강도 실험: current ${lo} → ${hi} (${step} 씩)"
        echo "  각 단계마다 상자를 들어올려 5초 버티는지 보세요."
        echo "  ── 미끄러지지 않는 최소값이 나오면 Ctrl+C 로 멈추고 그 값을 기록하세요."
        echo
        for c in $(seq "$lo" "$step" "$hi"); do
            echo "  [current $c]"
            cmd 750 200          # 열기
            sleep 2
            read -rp "    상자를 손가락 사이에 놓고 Enter (건너뛰려면 s+Enter): " ans
            [[ "$ans" == "s" ]] && continue
            cmd 0 "$c"           # 그 힘으로 닫기
            sleep 1
            echo "    → 지금 상자를 들어올려 보세요. 미끄러지나요?"
            read -rp "    결과 [y=버팀 / n=미끄러짐 / q=종료]: " r
            case "$r" in
                y) echo "    ✅ current $c 에서 버팀 — 이 값을 docs 에 기록하세요"; ;;
                q) echo "    종료"; cmd 750 200; exit 0 ;;
                *) echo "    ❌ current $c 에서 미끄러짐 — 더 올립니다" ;;
            esac
            echo
        done
        cmd 750 200
        ;;

    *)
        sed -n '2,16p' "$0" | sed 's/^# \?//'
        exit 1
        ;;
esac
