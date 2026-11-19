"""Receive RobStride winding temperatures without transmitting on SocketCAN."""
import socket
import struct
import threading
import time


WARNING_C = 70.
STOP_C = 80.
FRESH_S = 2.
CAN_EFF = 0x80000000
STATUS_MASK = 0x1F0000FF
STATUS_FRAME = 0x020000FD
FRAME = struct.Struct("=IB3x8s")


def decode_temperature(frame):
    if len(frame) != FRAME.size:
        return None
    can_id, length, data = FRAME.unpack(frame)
    if length != 8 or can_id & 0xE0000000 != CAN_EFF or can_id & STATUS_MASK != STATUS_FRAME:
        return None
    axis = (can_id >> 8) & 0xFF
    if not 1 <= axis <= 6:
        return None
    return axis, int.from_bytes(data[6:8], "big") / 10, bool(can_id & (1 << 18))


class MotorTemperature:
    def __init__(self, channel="can0"):
        self.channel = channel
        self.lock = threading.Lock()
        self.readings = {}
        self.closed = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._listen, name="motor-temperature", daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=.5)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {
                axis: (reading[0], reading[1])
                for axis, reading in self.readings.items()
                if now-reading[2] <= FRESH_S
            }

    def drives_in_reset(self):
        now = time.monotonic()
        with self.lock:
            return len(self.readings) == 6 and all(
                mode == 0 and now-stamp <= FRESH_S
                for _, _, stamp, mode in self.readings.values()
            )

    def _listen(self):
        while not self.closed.is_set():
            try:
                with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                    # CAN_ERR_FLAG in a socket filter selects the error-frame filter list.
                    mask = 0xDF00FFFF
                    filters = b"".join(
                        struct.pack("=II", CAN_EFF | STATUS_FRAME | (axis << 8), mask)
                        for axis in range(1, 7)
                    )
                    bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
                    bus.bind((self.channel,))
                    bus.settimeout(.2)
                    while not self.closed.is_set():
                        try:
                            frame = bus.recv(FRAME.size)
                            value = decode_temperature(frame)
                        except socket.timeout:
                            continue
                        if value:
                            axis, temp, fault = value
                            mode = (FRAME.unpack(frame)[0] >> 22) & 3
                            with self.lock:
                                self.readings[axis] = (temp, fault, time.monotonic(), mode)
            except (AttributeError, OSError):
                self.closed.wait(1.)
