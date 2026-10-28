"""Read-only camera monitoring and at-most-once, explicitly requested commands."""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, HTTPRedirectHandler, ProxyHandler


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class Operation:
    def __init__(self, action: str, key: str | None = None) -> None:
        self.done = threading.Event()
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._data = dict(state="pending", ok=False, message="等待相机结果", command_id=None, key=key, action=action)

    def cancel(self) -> None:
        with self._lock:
            self._cancel.set()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._data)

    def _update(self, **values: Any) -> None:
        with self._lock:
            self._data.update(values)

    def _finish(self, state: str, message: str, ok: bool = False) -> None:
        self._update(state=state, message=message, ok=ok)
        self.done.set()


class CameraBridgeClient:
    def __init__(self, journal_path: Path, *, url: str = "http://127.0.0.1:8023",
                 dry_run: bool = False, on_update=None, poll_interval: float = .5,
                 request_timeout: float = 1.5, result_timeout: float = 8.0) -> None:
        parsed = urlsplit(url)
        if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost") or parsed.path not in ("", "/") or parsed.query or parsed.fragment or parsed.username:
            raise ValueError("camera bridge must use a local HTTP address")
        for value in (poll_interval, request_timeout, result_timeout):
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("camera timeouts must be positive finite numbers")
        self.url = url.rstrip("/")
        self.journal_path = Path(journal_path)
        self.dry_run, self.on_update = dry_run, on_update
        self.poll_interval, self.request_timeout, self.result_timeout = poll_interval, request_timeout, result_timeout
        self._lock = threading.RLock()
        self._refresh_lock = threading.Lock()
        self._stopped = threading.Event()
        self._poller: threading.Thread | None = None
        self._worker: threading.Thread | None = None
        self._operation: Operation | None = None
        self._seen = 0.
        self._raw: dict[str, Any] = {}
        self._online = False
        self._error = "尚未读取相机状态"
        self._review = False
        self._consumed: set[str] = set()
        latest: dict[str, str] = {}
        if self.journal_path.exists():
            for line in self.journal_path.read_text(encoding="utf-8").splitlines():
                record = json.loads(line)
                if not isinstance(record, dict) or not isinstance(record.get("key"), str) or record.get("type") not in ("attempt", "result", "review_clear"):
                    raise ValueError("invalid camera attempt journal")
                if record["type"] == "review_clear":
                    latest.clear()
                    continue
                self._consumed.add(record["key"])
                latest[record["key"]] = record.get("state", "unknown")
            self._review = any(state in ("pending", "unknown") for state in latest.values())
        if dry_run:
            self._raw = {"device_online": True, "device": {"connected": False, "busy": False,
                         "camera_name": "模拟相机", "firmware": "dry-run"}}
            self._seen, self._online, self._error = time.monotonic(), True, ""

    def _notify(self) -> None:
        if self.on_update is not None:
            try:
                self.on_update(self.snapshot())
            except Exception:
                pass

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            fresh = time.monotonic() - self._seen <= 2
            online = self._online and fresh and not self._stopped.is_set()
            device = self._raw.get("device") or {}
            device_online = online and bool(self._raw.get("device_online"))
            connected = device_online and bool(device.get("connected"))
            own_busy = self._operation is not None and not self._operation.done.is_set()
            busy = own_busy or bool(self._raw.get("pending")) or bool(device.get("busy"))
            return {"bridge_online": online, "device_online": device_online,
                    "connected": connected, "busy": busy,
                    "ready": connected and not busy and not self._review,
                    "camera_name": str(device.get("camera_name") or ""),
                    "firmware": str(device.get("firmware") or ""),
                    "error": self._error if fresh else "相机桥状态已过期",
                    "operation": self._operation.snapshot() if self._operation else None,
                    "requires_review": self._review}

    def start(self) -> None:
        with self._lock:
            if self._stopped.is_set():
                raise RuntimeError("camera bridge is closed")
            if self._poller and self._poller.is_alive():
                return
            self._poller = threading.Thread(target=self._poll, name="camera-status", daemon=True)
            self._poller.start()

    def close(self) -> None:
        self._stopped.set()
        with self._lock:
            if self._operation and not self._operation.done.is_set():
                self._operation.cancel()
        for thread in (self._poller, self._worker):
            if thread and thread is not threading.current_thread():
                thread.join(timeout=self.request_timeout + .5)
        self._notify()

    def _poll(self) -> None:
        while not self._stopped.is_set():
            self.refresh()
            self._stopped.wait(self.poll_interval)

    def _http(self, method: str, path: str, payload=None) -> dict[str, Any]:
        headers = {"Accept": "application/json"}
        body = None
        if payload is not None:
            body = json.dumps(payload).encode()
            headers.update({"Content-Type": "application/json", "Origin": self.url,
                            "X-Shutter-Client": "1"})
        request = Request(self.url + path, data=body, headers=headers, method=method)
        with build_opener(ProxyHandler({}), _NoRedirect()).open(request, timeout=self.request_timeout) as response:
            if response.status != (202 if method == "POST" else 200):
                raise ValueError("unexpected camera bridge response")
            result = json.loads(response.read(65537))
            if not isinstance(result, dict):
                raise ValueError("invalid camera bridge response")
            return result

    def refresh(self) -> dict[str, Any]:
        if self._stopped.is_set():
            return self.snapshot()
        with self._refresh_lock:
            try:
                if self.dry_run:
                    with self._lock:
                        raw = self._raw
                else:
                    raw = self._http("GET", "/api/status")
                    if not isinstance(raw.get("device", {}), dict):
                        raise ValueError("invalid camera device state")
                with self._lock:
                    self._raw, self._online, self._seen = raw, True, time.monotonic()
                    self._error = str(raw.get("error") or "")
            except Exception as exc:
                with self._lock:
                    self._online = False
                    self._error = f"相机桥连接失败：{exc}"
        self._notify()
        return self.snapshot()

    def _journal(self, record: dict[str, Any]) -> None:
        self.journal_path.parent.mkdir(parents=True, exist_ok=True)
        with self.journal_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"time": time.time(), **record}, ensure_ascii=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def command(self, action: str) -> Operation:
        if action not in ("connect", "disconnect"):
            raise ValueError("only manual connect/disconnect commands are supported")
        return self._begin(action)

    def start_shutter(self, key: str, cancel_event: threading.Event | None = None) -> Operation:
        if not isinstance(key, str) or not key.strip() or len(key) > 256:
            raise ValueError("photo attempt key must contain 1-256 characters")
        return self._begin("shutter", key, cancel_event)

    def cancel_photo(self) -> None:
        with self._lock:
            if self._operation and self._operation.snapshot()["action"] == "shutter" and not self._operation.done.is_set():
                self._operation.cancel()

    def _begin(self, action: str, key: str | None = None, cancel_event=None) -> Operation:
        operation = Operation(action, key)
        with self._lock:
            if self._stopped.is_set():
                operation._finish("failed", "相机桥已关闭")
                return operation
            if self._operation and not self._operation.done.is_set():
                operation._finish("failed", "相机操作正在进行")
                return operation
            if key is not None and key in self._consumed:
                operation._finish("failed", "此拍摄尝试已使用，禁止重复发送")
                return operation
            if action == "shutter" and self._review:
                operation._finish("failed", "上次拍摄结果未知，请人工检查并重新连接")
                return operation
            state = self.snapshot()
            if not state["bridge_online"] or not state["device_online"]:
                operation._finish("failed", "相机或 USB 状态离线/过期")
                return operation
            if state["busy"]:
                operation._finish("failed", "相机正在忙碌")
                return operation
            if action == "shutter" and not state["connected"]:
                operation._finish("failed", "请先手动连接相机")
                return operation
            self._operation = operation
            self._worker = threading.Thread(target=self._execute, args=(operation, cancel_event),
                                            name="camera-command", daemon=True)
            self._worker.start()
        self._notify()
        return operation

    def _execute(self, operation: Operation, external_cancel) -> None:
        sent = False
        intent = False
        state, message, ok = "failed", "相机操作失败", False
        def cancelled():
            return operation._cancel.is_set() or self._stopped.is_set() or (external_cancel is not None and external_cancel.is_set())
        action, key = operation.snapshot()["action"], operation.snapshot()["key"]
        try:
            if cancelled():
                state, message = "cancelled", "操作在发送前取消"
                return
            if key is not None:
                with self._lock:
                    if cancelled():
                        state, message = "cancelled", "操作在发送前取消"
                        return
                    self._journal({"type": "attempt", "key": key, "action": action, "state": "pending"})
                    self._consumed.add(key)
                    intent = True
            if cancelled():
                state, message = "cancelled", "操作在发送前取消"
                return
            if self.dry_run:
                with self._lock:
                    self._raw["device"]["connected"] = action != "disconnect"
                    self._seen = time.monotonic()
                state, message, ok = "completed", "模拟相机操作完成", True
                return
            # This is the command handoff boundary. Cancellation before it forbids
            # POST; afterwards cancellation cannot undo delivery and means unknown.
            with operation._lock:
                if cancelled():
                    state, message = "cancelled", "操作在发送前取消"
                    return
                sent = True
            response = self._http("POST", "/api/command", {"action": action})
            ident = response.get("id")
            if not isinstance(ident, str) or not ident:
                raise ValueError("相机桥未返回命令 ID，结果未知")
            operation._update(command_id=ident)
            deadline = time.monotonic() + self.result_timeout
            while time.monotonic() < deadline:
                if cancelled():
                    state, message = "unknown", "指令发送后取消，拍摄结果未知；未重试"
                    return
                self.refresh()
                if cancelled():
                    state, message = "unknown", "指令发送后取消，拍摄结果未知；未重试"
                    return
                with self._lock:
                    candidates = [self._raw.get("last_result"), (self._raw.get("device") or {}).get("last_result")]
                    result = next((r for r in candidates if isinstance(r, dict) and r.get("id") == ident), None)
                if result is not None:
                    ok = result.get("ok") is True
                    message = str(result.get("message") or ("相机操作完成" if ok else "相机操作失败"))
                    state = "completed" if ok else "failed"
                    if not ok and ("未知" in message or "unknown" in message.lower()):
                        state = "unknown"
                    return
                self._stopped.wait(min(self.poll_interval, max(0., deadline-time.monotonic())))
            state, message = "unknown", "相机结果等待超时，未重试"
        except HTTPError as exc:
            # Bridge rejects these before accepting a command; network failures stay unknown.
            if exc.code in (400, 403, 404, 409):
                state, message = "failed", f"相机桥拒绝指令（{exc.code}）"
                try:
                    detail = json.loads(exc.read(65537)).get("error", "")
                    message = str(detail or message)
                    if "未知" in message or "unknown" in message.lower():
                        state = "unknown"
                except Exception:
                    pass
            else:
                state, message = ("unknown" if sent else "failed"), str(exc)
        except Exception as exc:
            state, message = ("unknown" if sent else "failed"), str(exc)
        finally:
            with self._lock:
                if state == "unknown":
                    self._review = True
                elif action == "connect" and ok:
                    try:
                        self._journal({"type": "review_clear", "key": "", "state": "completed"})
                        self._review = False
                    except Exception:
                        self._review = True
                        state, message, ok = "unknown", "连接确认日志写入失败", False
                if intent:
                    try:
                        self._journal({"type": "result", "key": key, "action": action,
                                       "state": state, "command_id": operation.snapshot()["command_id"]})
                    except Exception:
                        self._review = True
                        state, message, ok = "unknown", "相机结果日志写入失败，请人工检查", False
                operation._finish(state, message, ok)
            self._notify()
