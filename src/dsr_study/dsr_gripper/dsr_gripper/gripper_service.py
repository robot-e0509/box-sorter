#!/usr/bin/env python3
# =====================================================================
#  RH-P12-RN(A) 그리퍼 ROS2 서비스 노드  (상주 루프 · 고속)
#
#  서비스: /dsr01/gripper/cmd  (dsr_gripper_interfaces/GripperCmd)
#     요청: position(0~750), current(힘, 0이면 노드 기본값)
#     응답: success
#
#  구조:
#    - 최초 1회 DrlStart 로 '무한 루프 DRL' 을 띄움 → 시리얼 계속 열어둠 + init 1회
#    - 이후 매 명령: ROS2가 명령 레지스터에만 write (빠름)
#    - 루프는 seq 레지스터가 바뀌면 position/current 를 읽어 move
#
#  레지스터 코덱(실기 확인): 한 번 건널 때 f(x)=(x & 0xFF)<<24  → 한 칸당 8비트.
#    안전하게 7비트씩 분할(값 0~127, 부호문제 없음)해서 두 칸으로 전송.
#    받는 쪽 복원: byte = (v >> 24) & 0xFF
# =====================================================================

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from dsr_msgs2.srv import DrlStart, SetOutputRegisterInt
from dsr_gripper_interfaces.srv import GripperCmd

DRL_START_SRV = "/dsr01/dsr_controller2/drl/drl_start"
SET_REG_SRV = "/dsr01/dsr_controller2/plc/set_output_register_int"
GRIPPER_SRV = "/dsr01/gripper/cmd"

# 명령 레지스터 주소
REG_SEQ = 10
REG_POS_HI = 11
REG_POS_LO = 12
REG_CUR_HI = 13
REG_CUR_LO = 14

# --- 상주 루프 DRL 템플릿 -------------------------------------------
# %(...)s 로 slave_id / baudrate / 레지스터 주소 치환
DRL_LOOP_TMPL = r'''
g_slaveid = 0
set_Operating_Mode = 5
set_Torque_Enable  = 256
set_Goal_Current   = 275
set_Goal_Position  = 282

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

def dec(v):
    # ROS2가 쓴 7비트 값을 DRL이 읽으면 (b<<24) 형태 -> b 복원
    return (v >> 24) & 0xFF

flange_serial_open(baudrate=%(baud)d, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
modbus_set_slaveid(%(sid)d)
gripper_init()

last_seq = dec(get_output_register_int(%(seq)d))
while True:
    seq = dec(get_output_register_int(%(seq)d))
    if seq != last_seq:
        last_seq = seq
        ph = dec(get_output_register_int(%(ph)d))
        pl = dec(get_output_register_int(%(pl)d))
        ch = dec(get_output_register_int(%(ch)d))
        cl = dec(get_output_register_int(%(cl)d))
        pos = (ph << 7) | pl
        cur = (ch << 7) | cl
        set_gripper_current(cur)
        gripper_move(pos)
    wait(0.02)
'''


def build_loop_drl(slave_id, baudrate):
    return DRL_LOOP_TMPL % {
        "baud": baudrate, "sid": slave_id,
        "seq": REG_SEQ, "ph": REG_POS_HI, "pl": REG_POS_LO,
        "ch": REG_CUR_HI, "cl": REG_CUR_LO,
    }


class GripperService(Node):
    def __init__(self):
        super().__init__('gripper_service')
        self.declare_parameter('current', 200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('launch_wait', 3.0)   # 루프 기동 + init 대기(초)

        self.loop_started = False
        self.seq = 0
        cbg = ReentrantCallbackGroup()

        self.drl_cli = self.create_client(DrlStart, DRL_START_SRV, callback_group=cbg)
        self.reg_cli = self.create_client(SetOutputRegisterInt, SET_REG_SRV, callback_group=cbg)
        self.srv = self.create_service(GripperCmd, GRIPPER_SRV, self.on_cmd, callback_group=cbg)

        self.get_logger().info("그리퍼 서비스 준비됨: %s" % GRIPPER_SRV)

    def _i(self, n):
        return self.get_parameter(n).get_parameter_value().integer_value

    def _f(self, n):
        return self.get_parameter(n).get_parameter_value().double_value

    def _set_reg(self, address, value):
        if not self.reg_cli.wait_for_service(timeout_sec=2.0):
            return False
        r = SetOutputRegisterInt.Request()
        r.address = address
        r.value = value
        res = self.reg_cli.call(r)
        return res is not None and res.success

    def _launch_loop(self):
        if not self.drl_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("drl_start 서비스 없음")
            return False
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = build_loop_drl(self._i('slave_id'), self._i('baudrate'))
        # 무한 루프라 응답이 안 올 수 있으니 기다리지 않고 발사(fire-and-forget)
        self.drl_cli.call_async(req)
        time.sleep(self._f('launch_wait'))   # 루프 기동 + init 완료 대기
        self.get_logger().info("상주 루프 DRL 기동됨")
        return True

    def on_cmd(self, request, response):
        if not self.loop_started:
            if not self._launch_loop():
                response.success = False
                return response
            self.loop_started = True

        pos = max(0, min(750, request.position))
        # 이 그리퍼는 실제 stroke가 0=열림 / 750=닫힘 이라, 문서 규약(0=닫힘, 750=열림)에
        # 맞추기 위해 하드웨어로 보낼 때 반전한다.
        pos = 750 - pos
        cur = request.current if request.current > 0 else self._i('current')
        cur = max(0, min(16383, cur))   # 7비트x2 = 14비트 범위

        # 7비트 분할
        ph, pl = (pos >> 7) & 0x7F, pos & 0x7F
        ch, cl = (cur >> 7) & 0x7F, cur & 0x7F

        # payload 먼저 쓰고, seq 를 마지막에 써서 트리거
        self.seq = (self.seq + 1) & 0x7F
        ok = (self._set_reg(REG_POS_HI, ph) and self._set_reg(REG_POS_LO, pl)
              and self._set_reg(REG_CUR_HI, ch) and self._set_reg(REG_CUR_LO, cl)
              and self._set_reg(REG_SEQ, self.seq))

        response.success = ok
        if ok:
            self.get_logger().info("gripper cmd: pos=%d cur=%d seq=%d" % (pos, cur, self.seq))
        else:
            self.get_logger().error("레지스터 write 실패")
        return response


def main():
    rclpy.init()
    node = GripperService()
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
