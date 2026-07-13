#!/usr/bin/env python3
# =====================================================================
#  RH-P12-RN(A) 그리퍼 ROS2 서비스 노드  (방식 A · 백업용 · 원본 동작)
#
#  ※ 기본 고속 노드는 gripper_service (상주 루프, 방식 B) 입니다.
#    이 파일은 "매 호출마다 open→move→close" 하는 원래 방식 A의 백업본으로,
#    검증된 원본 동작(닫기 전 wait 유지)을 그대로 보존합니다.
#
#  서비스: /dsr01/gripper/cmd  (dsr_gripper_interfaces/GripperCmd)  ← 방식 B와 동일
#     요청: position(0~750), current(힘, 0이면 노드 기본값)
#     응답: success
#
#  경로: ROS2 --DrlStart--> 컨트롤러 DRL --RS485/Modbus--> 그리퍼
#    매 호출마다 시리얼 open -> (최초 1회 init) -> current -> move -> close.
#    단순하지만 호출당 지연 큼(open/close + 정착 wait).
# =====================================================================

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from dsr_msgs2.srv import DrlStart
from dsr_gripper_interfaces.srv import GripperCmd

DRL_START_SRV = "/dsr01/dsr_controller2/drl/drl_start"
GRIPPER_SRV = "/dsr01/gripper/cmd"

# --- DRL 헬퍼 (쓰기 FC06/FC16) ---------------------------------------
# grap_bottle_working.py 기준. modbus_send_make 는 표준 Modbus-RTU
# CRC16(poly 0xA001, init 0xFFFF, low byte first) 으로 복원.
# ※ gripper_move 의 wait(1) 은 flange_serial_close() 전 정착 시간으로 반드시 필요.
DRL_HELPERS = r'''
g_slaveid = 0
set_Operating_Mode   = 5
set_Torque_Enable    = 256
set_Goal_Current     = 275
set_Goal_Position    = 282

def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid

def modbus_crc(data):
    crc = 0xFFFF
    for b in data:
        crc = crc ^ b
        i = 0
        while i < 8:
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
            i = i + 1
    return crc

def modbus_send_make(data):
    crc = modbus_crc(data)
    data += (crc & 0xFF).to_bytes(1, byteorder='big')
    data += ((crc >> 8) & 0xFF).to_bytes(1, byteorder='big')
    return data

def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)

def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2*cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)

def recv_check():
    size, val = flange_serial_read(0.1)
    return (size > 0), val

def gripper_init():
    flange_serial_write(modbus_fc06(set_Torque_Enable, 0))
    recv_check()
    wait(0.2)
    flange_serial_write(modbus_fc06(set_Operating_Mode, 5 << 8))
    recv_check()
    wait(0.2)
    flange_serial_write(modbus_fc06(set_Torque_Enable, 1))
    recv_check()
    wait(0.2)

def set_gripper_current(current):
    flange_serial_write(modbus_fc06(set_Goal_Current, current))
    recv_check()

def gripper_move(stroke):
    flange_serial_write(modbus_fc16(set_Goal_Position, 2, [stroke, 0]))
    recv_check()
    wait(1)
'''


def build_drl(position, current, slave_id, baudrate, do_init):
    """방식 A(제어 전용): open -> (init) -> current -> move -> close."""
    lines = [DRL_HELPERS]
    lines.append("flange_serial_open(baudrate=%d, bytesize=DR_EIGHTBITS, "
                 "parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)" % baudrate)
    lines.append("modbus_set_slaveid(%d)" % slave_id)
    if do_init:
        lines.append("gripper_init()")
    lines.append("set_gripper_current(%d)" % current)
    lines.append("wait(0.2)")
    lines.append("gripper_move(%d)" % position)
    lines.append("flange_serial_close()")
    return "\n".join(lines)


class GripperServiceA(Node):
    def __init__(self):
        super().__init__('gripper_service_a')
        self.declare_parameter('current', 200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('baudrate', 57600)

        self.initialized = False
        cbg = ReentrantCallbackGroup()

        self.drl_cli = self.create_client(DrlStart, DRL_START_SRV, callback_group=cbg)
        self.srv = self.create_service(GripperCmd, GRIPPER_SRV, self.on_cmd,
                                       callback_group=cbg)

        self.get_logger().info("그리퍼 서비스(방식 A · 백업) 준비됨: %s" % GRIPPER_SRV)

    def _i(self, n):
        return self.get_parameter(n).get_parameter_value().integer_value

    def on_cmd(self, request, response):
        target = max(0, min(750, request.position))
        # 이 그리퍼는 실제 stroke가 0=열림 / 750=닫힘 이라, 문서 규약(0=닫힘, 750=열림)에
        # 맞추기 위해 하드웨어로 보낼 때 반전한다.
        target = 750 - target
        current = request.current if request.current > 0 else self._i('current')
        do_init = not self.initialized

        drl = build_drl(target, current, self._i('slave_id'),
                        self._i('baudrate'), do_init)

        if not self.drl_cli.wait_for_service(timeout_sec=3.0):
            response.success = False
            self.get_logger().error("drl_start 서비스 없음")
            return response

        req = DrlStart.Request()
        req.robot_system = 0
        req.code = drl
        result = self.drl_cli.call(req)

        if result is None or not result.success:
            response.success = False
            self.get_logger().error("DrlStart 실패 (auto 모드 + STANDBY 확인)")
            return response

        self.initialized = True
        response.success = True
        self.get_logger().info("gripper move: position=%d current=%d init=%s"
                               % (target, current, do_init))
        return response


def main():
    rclpy.init()
    node = GripperServiceA()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
