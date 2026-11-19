"""Serve the original UI and its compatibility API; startup is non-actuating."""
import argparse
import json
import mimetypes
import os
import queue
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from .app import Workbench
from .ros_backend import RosBackend


class Handler(BaseHTTPRequestHandler):
    def json(self, payload, code=200):
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/status":
            self.json(self.server.app.snapshot())
        elif path == "/api/events":
            q = queue.Queue(maxsize=64)
            self.server.app.subscribers.append(q)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    try:
                        event = q.get(timeout=5)
                        self.wfile.write(("data: "+json.dumps(event,ensure_ascii=False)+"\n\n").encode())
                    except queue.Empty:
                        self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionError):
                pass
            finally:
                self.server.app.subscribers.remove(q)
        else:
            root = self.server.static.resolve()
            target = (root / ("index.html" if path == "/" else path.lstrip("/"))).resolve()
            if not target.is_relative_to(root) or not target.is_file():
                self.send_error(404)
                return
            data = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(str(target))[0] or "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 <= length <= 65536:
                raise ValueError("请求过大")
            data = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(data, dict):
                raise ValueError("请求必须是对象")
            self.json(self.server.app.command(urlsplit(self.path).path, data))
        except Exception as exc:
            self.json(dict(error=str(exc)), 400)


def main():
    from .motor_temperature import MotorTemperature
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8012)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--home", type=Path, required=True)
    parser.add_argument("--static", type=Path, required=True)
    parser.add_argument("--rs-root", type=Path, required=True)
    args = parser.parse_args()
    def launcher():
        # Called only by an explicitly submitted and authorized /api/start.
        env = dict(os.environ, REBOTARM_RS_HARDWARE_CONFIRM="I_UNDERSTAND_RS_WILL_MOVE")
        with (args.data/"hardware-start.log").open("a") as out:
            proc = subprocess.Popen([str(args.rs_root/"scripts/start_camera_hardware.sh")],
                cwd=args.rs_root, env=env, stdin=subprocess.DEVNULL, stdout=out,
                stderr=subprocess.STDOUT, start_new_session=True)
        (args.data/"hardware-launch.pid").write_text(str(proc.pid))
    backend = RosBackend()
    app = Workbench(args.data,args.home,backend,launcher=launcher,temperature=MotorTemperature())
    http = ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
    http.daemon_threads = True
    http.app, http.static = app, args.static
    app.start_monitor()
    print(f"旧工作台界面已恢复：http://127.0.0.1:{args.port}/ · 新RS底层 · 未启动机械臂",flush=True)
    try:
        http.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
        http.server_close()


if __name__ == "__main__":
    main()
