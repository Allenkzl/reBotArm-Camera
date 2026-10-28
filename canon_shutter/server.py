#!/usr/bin/env python3
"""Local web -> USB JSON -> XIAO BLE bridge. Never retries camera commands."""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import serial
from serial.tools import list_ports

STATIC = Path(__file__).parent / "static"
ACTIONS = {"pair", "scan", "connect", "disconnect", "forget", "shutter"}


class Bridge:
    def __init__(self, port, serial_number):
        self.requested_port = port
        self.serial_number = serial_number
        self.port = port
        self.lock = threading.RLock()
        self.connection = None
        self.device = {}
        self.last_seen = 0
        self.last_result = None
        self.last_device_result_id = ""
        self.pending = None
        self.pending_since = 0
        self.events = deque(maxlen=50)
        self.stopped = threading.Event()
        self.last_error = "等待 USB 设备"

    def event(self, text):
        # Caller owns lock.
        if self.events and self.events[-1]["message"] == text:
            return
        self.events.append({"time": time.strftime("%H:%M:%S"), "message": text})

    def online(self):
        return self.connection is not None and time.monotonic() - self.last_seen < 3

    def snapshot(self):
        with self.lock:
            online = self.online()
            return {"device_online": online, "port": self.port,
                    "device": self.device if online else {},
                    "last_result": self.last_result,
                    "pending": self.pending, "events": list(self.events),
                    "error": "" if online else self.last_error}

    def send(self, action, address=""):
        if action not in ACTIONS:
            raise ValueError("未知操作")
        if address and action != "connect":
            raise ValueError("此操作不接受相机地址")
        with self.lock:
            if not self.online():
                raise RuntimeError("ESP32 离线或状态已过期，请检查 USB 连接")
            if self.pending or self.device.get("busy"):
                raise RuntimeError("设备忙碌，请等待当前操作结束")
            if action == "shutter" and not self.device.get("connected"):
                raise RuntimeError("请先连接相机")
            if action == "connect" and address and not any(
                    c.get("address") == address for c in self.device.get("candidates", [])):
                raise ValueError("请从本次搜索结果选择相机")
            ident = uuid.uuid4().hex
            command = {"id": ident, "action": action, "address": address}
            self.pending = ident
            self.pending_since = time.monotonic()
            try:
                self.connection.write((json.dumps(command) + "\n").encode())
                self.connection.flush()
            except (serial.SerialException, OSError) as error:
                self.pending = None
                self.last_seen = 0
                self.last_error = "USB 写入失败；操作结果未知，未自动重试"
                raise RuntimeError(self.last_error) from error
            self.event({"pair": "开始配对新相机", "scan": "搜索相机", "connect": "连接相机",
                        "disconnect": "断开相机", "forget": "清除本机配对", "shutter": "发送一次快门"}[action])
            return ident

    def target_port(self):
        if not self.serial_number:
            return self.requested_port
        for port in list_ports.comports():
            if (port.vid == 0x303A and port.pid == 0x1001 and
                    (port.serial_number or "").lower() == self.serial_number.lower()):
                return port.device
        raise serial.SerialException("未找到指定的 XIAO，请检查 USB 连接")

    def consume(self, line):
        try:
            data = json.loads(line)
        except (ValueError, UnicodeError):
            return
        if not isinstance(data, dict):
            return
        with self.lock:
            if data.get("type") == "status" and data.get("firmware", "").startswith("canon-shutter-"):
                old_uptime = self.device.get("uptime_ms", 0)
                if data.get("uptime_ms", 0) < old_uptime and self.pending:
                    self.last_result = {"id": self.pending, "ok": False,
                                        "message": "设备已重启，操作结果未知；未自动重试"}
                    self.pending = None
                self.last_seen = time.monotonic()
                self.last_error = "设备状态已过期，请检查 USB 连接"
                self.device = data
                self.event(data.get("message", "状态更新"))
                completed = data.get("last_result")
                if (isinstance(completed, dict) and completed.get("id") and
                        completed["id"] != self.last_device_result_id):
                    self.last_device_result_id = completed["id"]
                    self.last_result = completed
                    if completed["id"] == self.pending:
                        self.pending = None
            elif data.get("type") == "result":
                self.last_result = data
                self.last_device_result_id = data.get("id", "")
                if self.pending == data.get("id"):
                    self.pending = None
                self.event(data.get("message", "操作结束"))
            elif data.get("type") == "fatal":
                self.last_error = data.get("message", "设备初始化失败")
                self.last_seen = 0
                self.event(self.last_error)

    def run(self):
        while not self.stopped.is_set():
            connection = None
            try:
                target = self.target_port()
                connection = serial.Serial(port=None, baudrate=115200, timeout=0.3, write_timeout=2)
                connection.dtr = False
                connection.rts = False
                connection.port = target
                connection.open()
                with self.lock:
                    self.connection = connection
                    self.port = target
                    self.last_seen = 0
                    self.event("USB 已接通，等待固件状态")
                buffer = bytearray()
                while not self.stopped.is_set():
                    chunk = connection.read(1024)
                    buffer.extend(chunk)
                    while b"\n" in buffer:
                        line, _, rest = buffer.partition(b"\n")
                        buffer = bytearray(rest)
                        self.consume(line)
                    if len(buffer) > 16384:
                        buffer.clear()
                    with self.lock:
                        if self.pending and time.monotonic() - self.pending_since > 60:
                            self.last_result = {"id": self.pending, "ok": False,
                                                "message": "操作等待超时，结果未知；未自动重试"}
                            self.event(self.last_result["message"])
                            self.pending = None
            except (serial.SerialException, OSError) as error:
                with self.lock:
                    self.last_error = f"USB 未连接：{error}"
                    self.event("USB 已断开，请检查设备连接")
            finally:
                with self.lock:
                    self.connection = None
                    self.last_seen = 0
                    self.device = {}
                    if self.pending:
                        self.last_result = {"id": self.pending, "ok": False,
                                            "message": "USB 已断开，操作结果未知；未自动重试"}
                        self.pending = None
                if connection is not None:
                    connection.close()
            self.stopped.wait(1)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        if self.path == "/api/status" and args[1] == "200":
            return
        print(time.strftime("%H:%M:%S"), fmt % args, flush=True)

    def respond(self, code, payload, mime="application/json; charset=utf-8"):
        body = json.dumps(payload, ensure_ascii=False).encode() if isinstance(payload, dict) else payload
        self.send_response(code)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def valid_host(self):
        return self.headers.get("Host") in {
            f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}

    def do_GET(self):
        if not self.valid_host():
            self.respond(403, {"error": "无效访问地址"})
            return
        path = urlsplit(self.path).path
        if path == "/api/status":
            self.respond(200, self.server.bridge.snapshot())
            return
        files = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                 "/style.css": ("style.css", "text/css; charset=utf-8")}
        if path not in files:
            self.respond(404, {"error": "页面不存在"})
            return
        name, mime = files[path]
        self.respond(200, (STATIC / name).read_bytes(), mime)

    def do_POST(self):
        expected = f"http://{self.headers.get('Host')}"
        if (not self.valid_host() or self.headers.get("Origin", expected) != expected or
                self.headers.get("X-Shutter-Client") != "1"):
            self.respond(403, {"error": "请通过本机快门网页操作"})
            return
        if self.path != "/api/command":
            self.respond(404, {"error": "接口不存在"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 1024 or self.headers.get_content_type() != "application/json":
                raise ValueError("请求格式错误")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict) or not isinstance(data.get("action"), str) or not isinstance(data.get("address", ""), str):
                raise ValueError("请求格式错误")
            ident = self.server.bridge.send(data["action"], data.get("address", ""))
        except (ValueError, KeyError) as error:
            self.respond(400, {"error": str(error)})
        except RuntimeError as error:
            self.respond(409, {"error": str(error)})
        else:
            self.respond(202, {"id": ident, "message": "操作已提交，等待设备结果"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem11301")
    parser.add_argument("--serial-number", default="28:84:85:B4:0A:34")
    parser.add_argument("--http-port", type=int, default=8023)
    args = parser.parse_args()
    bridge = Bridge(args.port, args.serial_number)
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    server.bridge = bridge
    threading.Thread(target=bridge.run, daemon=True, name="xiao-usb").start()
    print(f"Canon Shutter: http://127.0.0.1:{args.http_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stopped.set()
        server.server_close()


if __name__ == "__main__":
    main()
