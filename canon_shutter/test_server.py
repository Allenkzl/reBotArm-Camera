"""Check command gating, no-retry semantics, and browser request boundaries."""
import json
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener
from http.server import ThreadingHTTPServer

from server import Bridge, Handler


class Transport:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(json.loads(data))
        return len(data)

    def flush(self):
        pass


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = Bridge("unused", "")
        self.transport = Transport()
        self.bridge.connection = self.transport
        self.status(connected=True, uptime_ms=10000)

    def status(self, **extra):
        self.bridge.consume(json.dumps({"type": "status", "firmware": "canon-shutter-test",
                                        "busy": False, **extra}).encode())

    def test_stale_status_never_sends(self):
        self.bridge.last_seen = time.monotonic() - 4
        with self.assertRaises(RuntimeError):
            self.bridge.send("shutter")
        self.assertEqual(self.transport.writes, [])
        self.assertEqual(self.bridge.snapshot()["device"], {})

    def test_disconnected_camera_never_sends(self):
        self.status(connected=False)
        with self.assertRaises(RuntimeError):
            self.bridge.send("shutter")
        self.assertEqual(self.transport.writes, [])

    def test_duplicate_click_and_status_poll_do_not_repeat_shutter(self):
        ident = self.bridge.send("shutter")
        with self.assertRaises(RuntimeError):
            self.bridge.send("shutter")
        self.bridge.snapshot()
        self.status(connected=True, uptime_ms=11000)
        self.assertEqual(len(self.transport.writes), 1)
        self.bridge.consume(json.dumps({"type": "result", "id": ident, "ok": True}).encode())
        self.assertIsNone(self.bridge.pending)

    def test_device_reboot_does_not_replay_pending_shutter(self):
        ident = self.bridge.send("shutter")
        self.status(connected=False, uptime_ms=50)
        self.assertIsNone(self.bridge.pending)
        self.assertEqual(self.bridge.last_result["id"], ident)
        self.assertFalse(self.bridge.last_result["ok"])
        self.assertEqual(len(self.transport.writes), 1)

    def test_lost_completion_frame_recovers_from_heartbeat(self):
        ident = self.bridge.send("shutter")
        self.status(connected=True, uptime_ms=11000,
                    last_result={"id": ident, "ok": True, "message": "快门指令已发送"})
        self.assertIsNone(self.bridge.pending)
        self.assertTrue(self.bridge.last_result["ok"])
        self.assertEqual(len(self.transport.writes), 1)

    def test_old_heartbeat_does_not_hide_new_timeout_result(self):
        old = {"id": "older", "ok": True, "message": "已完成"}
        self.status(connected=True, uptime_ms=11000, last_result=old)
        self.bridge.last_result = {"id": "newer", "ok": False, "message": "结果未知"}
        self.status(connected=True, uptime_ms=12000, last_result=old)
        self.assertEqual(self.bridge.last_result["id"], "newer")

    def test_write_failure_does_not_retry_or_claim_success(self):
        def broken(_):
            raise OSError("USB removed")
        self.transport.write = broken
        with self.assertRaises(RuntimeError):
            self.bridge.send("shutter")
        self.assertFalse(self.bridge.snapshot()["device_online"])
        self.assertIsNone(self.bridge.pending)

    def test_connect_only_accepts_scanned_address(self):
        with self.assertRaises(ValueError):
            self.bridge.send("connect", "00:11:22:33:44:55")
        self.assertEqual(self.transport.writes, [])


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.bridge = Bridge("unused", "")
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, headers):
        return self.opener.open(Request(self.url + "/api/command", data=b'{"action":"shutter"}',
                                        headers={"Content-Type": "application/json", **headers}))

    def test_cross_origin_cannot_trigger_camera(self):
        with self.assertRaises(HTTPError) as result:
            self.post({"X-Shutter-Client": "1", "Origin": "https://example.com"})
        self.assertEqual(result.exception.code, 403)

    def test_form_or_unmarked_request_cannot_trigger_camera(self):
        with self.assertRaises(HTTPError) as result:
            self.post({})
        self.assertEqual(result.exception.code, 403)

    def test_valid_request_to_offline_device_is_rejected(self):
        with self.assertRaises(HTTPError) as result:
            self.post({"X-Shutter-Client": "1", "Origin": self.url})
        self.assertEqual(result.exception.code, 409)

    def test_status_is_offline_and_static_page_served(self):
        with self.opener.open(self.url + "/api/status") as response:
            self.assertFalse(json.load(response)["device_online"])
        with self.opener.open(self.url) as response:
            self.assertIn("配对新相机".encode(), response.read())


if __name__ == "__main__":
    unittest.main()
