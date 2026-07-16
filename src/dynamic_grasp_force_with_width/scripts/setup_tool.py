"""툴 등록 + 상태 확인. 모션 없음."""
import time
import rclpy
import DR_init

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("setup_tool", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    set_robot_mode, get_robot_mode, get_current_posj,
    set_tool, set_tcp, get_tool, get_tcp,
    ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
)
from dsr_msgs2.srv import ConfigCreateTool, ConfigCreateTcp

# DSR_ROBOT2 는 import 시점에 서비스 클라이언트를 만들고, 호출할 때 wait_for_service 를
# 하지 않는다. spin_until_future_complete 에도 타임아웃이 없어서, 디스커버리가 끝나기
# 전에 부르면 영원히 매달린다. 여기서 한 번 기다려 준다.
# 이 드라이버에 아예 없는 서비스도 있으므로(예: motion/set_singularity_handling_force)
# 전부 기다리면 안 된다. 우리가 실제로 쓰는 것만 필수로 본다.
NEEDED = (
    "get_robot_mode", "set_robot_mode", "get_current_posj", "get_current_posx",
    "set_current_tool", "set_current_tcp", "get_current_tool", "get_current_tcp",
    "move_joint",
)
_need = [c for c in node.clients if any(k in c.srv_name for k in NEEDED)]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    _pending = [c for c in _need if not c.service_is_ready()]
    if not _pending:
        break
    rclpy.spin_once(node, timeout_sec=0.1)
if _pending:
    raise RuntimeError("서비스 디스커버리 실패: " + ", ".join(c.srv_name for c in _pending))
print(f"✓ 필수 서비스 {len(_need)}개 디스커버리 완료 ({time.time()-_t0:.1f}s)")

TOOL_NAME, TCP_NAME = "rh_p12_rn", "rh_p12_rn_tcp"
TOOL_WEIGHT = 0.5
TOOL_COG = [0.0, 0.0, 60.0]
TCP_POS = [0.0, 0.0, 150.0, 0.0, 0.0, 0.0]


def set_mode(mode, timeout=10.0):
    want = int(mode)
    if get_robot_mode() == want:
        return want
    set_robot_mode(want)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if get_robot_mode() == want:
            print(f"  모드 → {'MANUAL' if want == 0 else 'AUTONOMOUS'} ({time.time()-t0:.1f}s)")
            return want
        time.sleep(0.3)
    raise RuntimeError(f"모드 전환 실패 (현재 {get_robot_mode()}, 원한 값 {want})")


def _call(cli, req, what, timeout=10.0):
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"{what}: 서비스 없음")
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None:
        raise RuntimeError(f"{what}: 응답 없음 (timeout)")
    if not getattr(res, "success", True):
        raise RuntimeError(f"{what}: 거부됨 (success=False). manual 모드인지 확인하세요")
    return res


print("현재 모드:", get_robot_mode(), "(0=manual, 1=autonomous)")
print("현재 자세:", [round(v, 3) for v in get_current_posj()])
print("등록된 tool:", repr(get_tool()), " tcp:", repr(get_tcp()))

ns = f"/{ROBOT_ID}/dsr_controller2"
tool_cli = node.create_client(ConfigCreateTool, f"{ns}/tool/config_create_tool")
tcp_cli = node.create_client(ConfigCreateTcp, f"{ns}/tcp/config_create_tcp")

print("\n▸ 툴 등록 (manual 모드 필요)")
set_mode(ROBOT_MODE_MANUAL)

r = ConfigCreateTool.Request()
r.name, r.weight, r.cog, r.inertia = TOOL_NAME, TOOL_WEIGHT, TOOL_COG, [0.0] * 6
_call(tool_cli, r, "config_create_tool")

r = ConfigCreateTcp.Request()
r.name, r.pos = TCP_NAME, TCP_POS
_call(tcp_cli, r, "config_create_tcp")

set_tool(TOOL_NAME)
set_tcp(TCP_NAME)

set_mode(ROBOT_MODE_AUTONOMOUS)

tool, tcp = get_tool(), get_tcp()
if not tool or not tcp:
    raise RuntimeError(f"툴 등록 실패 (tool={tool!r}, tcp={tcp!r})")
print(f"\n✓ tool={tool}  tcp={tcp}")
print("✓ 모드:", get_robot_mode(), "(1=autonomous)")
print("✓ 자세:", [round(v, 3) for v in get_current_posj()])

rclpy.shutdown()
