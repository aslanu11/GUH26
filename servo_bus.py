"""
servo_bus.py - SHARED (owner: Aslan). Tiny raw driver for SO-101 arms (Feetech STS3215 servos).

Talks to the servos directly with feetech-servo-sdk, so it works on any laptop without LeRobot
and without re-calibrating. Positions are raw encoder ticks (0-4095 = one turn).

    pip install feetech-servo-sdk pyserial
"""
import threading
import time

from scservo_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM_JOINTS = JOINTS[:5]          # everything except the gripper / leader trigger
IDS = {n: i + 1 for i, n in enumerate(JOINTS)}

# STS3215 control table
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_TORQUE_LIMIT = 48
ADDR_PRESENT_POSITION = 56


def _decode(v):  # sign-magnitude, sign in bit 15
    return -(v & 0x7FFF) if v & 0x8000 else v


def _encode(v):
    v = int(round(v))
    return (abs(v) & 0x7FFF) | 0x8000 if v < 0 else v & 0x7FFF


class Bus:
    def __init__(self, port, baud=1_000_000):
        self.port_name = port
        self.port = PortHandler(port)
        self.ph = PacketHandler(0)
        if not self.port.openPort():
            raise IOError(f"cannot open serial port {port} (is MakerMods Lab still holding it?)")
        if not self.port.setBaudRate(baud):
            raise IOError(f"cannot set baud {baud} on {port}")
        self.lock = threading.Lock()
        self.sw = GroupSyncWrite(self.port, self.ph, ADDR_GOAL_POSITION, 2)

    def read_positions(self, retries=5):
        out = {}
        with self.lock:
            for name in JOINTS:
                for attempt in range(retries + 1):
                    val, res, _ = self.ph.read2ByteTxRx(self.port, IDS[name], ADDR_PRESENT_POSITION)
                    if res == COMM_SUCCESS:
                        out[name] = _decode(val)
                        break
                    time.sleep(0.01)
                    if attempt == retries:
                        raise IOError(f"no reply from servo {IDS[name]} ({name}) - power off or cable out?")
        return out

    def write_goals(self, goals):
        with self.lock:
            self.sw.clearParam()
            for name, val in goals.items():
                v = _encode(val)
                self.sw.addParam(IDS[name], [v & 0xFF, (v >> 8) & 0xFF])
            if self.sw.txPacket() != COMM_SUCCESS:
                raise IOError("goal write failed")

    def torque(self, on, names=None):
        with self.lock:
            for name in names or JOINTS:
                self.ph.write1ByteTxRx(self.port, IDS[name], ADDR_TORQUE_ENABLE, 1 if on else 0)

    def set_acceleration(self, value=40):
        with self.lock:
            for name in JOINTS:
                self.ph.write1ByteTxRx(self.port, IDS[name], ADDR_ACCELERATION, int(value))

    def set_torque_limit(self, name, value):
        with self.lock:
            self.ph.write2ByteTxRx(self.port, IDS[name], ADDR_TORQUE_LIMIT, int(value))

    def close(self):
        try:
            self.port.closePort()
        except Exception:
            pass


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [(p.device, p.description) for p in lp.comports()]
    except Exception:
        return []


def wait_s(t):
    time.sleep(t)
