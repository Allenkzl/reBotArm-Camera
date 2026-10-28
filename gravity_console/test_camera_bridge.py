import json
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

from gravity_console.camera_bridge import CameraBridgeClient


class CameraBridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'attempts.jsonl'
        self.posts = []
        self.mode = 'success'
        self.raw = {'device_online': True, 'device': {'connected': True, 'busy': False,
                    'camera_name': 'Canon', 'firmware': 'canon-shutter-0.1.1'}, 'last_result': None}
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def reply(self, code, data):
                body = json.dumps(data).encode()
                self.send_response(code); self.send_header('Content-Length', str(len(body))); self.end_headers()
                try: self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError): pass
            def do_GET(self):
                self.reply(200, owner.raw)
            def do_POST(self):
                owner.assertEqual(self.headers['X-Shutter-Client'], '1')
                owner.assertEqual(self.headers['Origin'], owner.url)
                action = json.loads(self.rfile.read(int(self.headers['Content-Length'])))['action']
                owner.posts.append(action)
                ident = f'cmd-{len(owner.posts)}'
                if owner.mode == 'lost':
                    time.sleep(.2)
                    self.reply(202, {'id': ident})
                    return
                if owner.mode == 'usbunknown':
                    self.reply(409, {'error': 'USB 写入失败，操作结果未知'})
                    return
                if owner.mode == 'success':
                    owner.raw['device']['connected'] = action != 'disconnect'
                    owner.raw['last_result'] = {'id': ident, 'ok': True, 'message': 'done'}
                elif owner.mode == 'mismatch':
                    owner.raw['last_result'] = {'id': 'other-id', 'ok': True}
                self.reply(202, {'id': ident})
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.client = self.make_client()
        self.addCleanup(self.client.close)

    def make_client(self, **kwargs):
        return CameraBridgeClient(self.path, url=self.url, poll_interval=.01,
                                  request_timeout=.06, result_timeout=.1, **kwargs)

    def done(self, op):
        self.assertTrue(op.done.wait(1), op.snapshot())
        return op.snapshot()

    def test_read_only_start_refresh_and_constructor(self):
        self.assertFalse(self.client.snapshot()['bridge_online'])
        self.client.refresh()
        self.client.start()
        time.sleep(.04)
        self.assertEqual(self.posts, [])
        self.assertTrue(self.client.snapshot()['ready'])

    def test_matching_results_and_persistent_duplicate_key(self):
        self.client.refresh()
        first = self.done(self.client.start_shutter('photo-1'))
        self.assertEqual(first['state'], 'completed')
        self.assertEqual(first['command_id'], 'cmd-1')
        self.assertFalse(self.done(self.client.start_shutter('photo-1'))['ok'])
        restarted = self.make_client()
        self.addCleanup(restarted.close)
        restarted.refresh()
        self.assertFalse(self.done(restarted.start_shutter('photo-1'))['ok'])
        self.assertTrue(self.done(restarted.start_shutter('photo-2'))['ok'])
        self.assertEqual(self.posts, ['shutter', 'shutter'])
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual([r['type'] for r in records], ['attempt', 'result', 'attempt', 'result'])

    def test_lost_202_consumes_key_never_retries_and_requires_review(self):
        self.client.refresh(); self.mode = 'lost'
        result = self.done(self.client.start_shutter('lost'))
        self.assertEqual(result['state'], 'unknown')
        self.assertTrue(self.client.snapshot()['requires_review'])
        self.assertFalse(self.client.snapshot()['ready'])
        self.done(self.client.start_shutter('new'))
        self.assertEqual(self.posts, ['shutter'])
        self.mode = 'success'
        self.assertTrue(self.done(self.client.command('connect'))['ok'])
        self.assertFalse(self.client.snapshot()['requires_review'])
        restarted = self.make_client(); self.addCleanup(restarted.close)
        self.assertFalse(restarted.snapshot()['requires_review'])
        restarted.refresh()
        self.assertFalse(self.done(restarted.start_shutter('lost'))['ok'])
        self.assertTrue(self.done(restarted.start_shutter('new'))['ok'])

    def test_unmatched_results_timeout_and_own_busy_is_preserved(self):
        self.client.refresh(); self.mode = 'mismatch'
        operation = self.client.start_shutter('mismatch')
        self.client.refresh()
        self.assertTrue(self.client.snapshot()['busy'])
        self.assertFalse(self.done(self.client.command('disconnect'))['ok'])
        self.assertEqual(self.done(operation)['state'], 'unknown')
        self.assertEqual(self.posts, ['shutter'])

    def test_cancel_before_post_never_writes_or_consumes(self):
        self.client.refresh()
        cancel = threading.Event(); cancel.set()
        self.assertEqual(self.done(self.client.start_shutter('cancel', cancel))['state'], 'cancelled')
        self.assertFalse(self.path.exists())
        self.assertEqual(self.posts, [])
        self.assertTrue(self.done(self.client.start_shutter('cancel'))['ok'])

    def test_inflight_cancel_is_unknown(self):
        self.client.refresh(); self.mode = 'pending'
        operation = self.client.start_shutter('inflight')
        deadline = time.monotonic() + 1
        while operation.snapshot()['command_id'] is None and time.monotonic() < deadline:
            time.sleep(.002)
        self.client.cancel_photo()
        self.assertEqual(self.done(operation)['state'], 'unknown')
        self.assertTrue(self.client.snapshot()['requires_review'])
        self.assertEqual(self.posts, ['shutter'])

    def test_cancel_during_result_refresh_cannot_claim_success(self):
        self.client.refresh()
        refresh = self.client.refresh
        def cancel_while_refreshing():
            self.client.cancel_photo()
            return refresh()
        with patch.object(self.client, 'refresh', side_effect=cancel_while_refreshing):
            operation = self.client.start_shutter('cancel-during-get')
            self.assertEqual(self.done(operation)['state'], 'unknown')
        self.assertEqual(self.posts, ['shutter'])
        self.assertTrue(self.client.snapshot()['requires_review'])

    def test_stale_and_device_offline_do_not_post(self):
        self.client.refresh()
        self.client._seen = time.monotonic() - 3
        self.assertFalse(self.done(self.client.start_shutter('stale'))['ok'])
        self.assertFalse(self.client.snapshot()['connected'])
        self.raw['device_online'] = False
        self.client.refresh()
        self.assertFalse(self.done(self.client.command('connect'))['ok'])
        self.assertEqual(self.posts, [])
        self.assertFalse(self.path.exists())

    def test_heartbeat_result_and_usb_unknown(self):
        self.client.refresh(); self.mode = 'pending'
        operation = self.client.start_shutter('heartbeat')
        deadline = time.monotonic() + 1
        while operation.snapshot()['command_id'] is None and time.monotonic() < deadline:
            time.sleep(.002)
        self.raw['device']['last_result'] = {'id': operation.snapshot()['command_id'], 'ok': True}
        self.assertTrue(self.done(operation)['ok'])
        self.mode = 'usbunknown'
        self.assertEqual(self.done(self.client.start_shutter('usb'))['state'], 'unknown')

    def test_journal_failure_prevents_post_and_notifies_failure(self):
        self.client.refresh()
        updates = []
        self.client.on_update = updates.append
        with patch.object(self.client, '_journal', side_effect=OSError('journal unavailable')):
            operation = self.client.start_shutter('journal-failure')
            self.assertEqual(self.done(operation)['state'], 'failed')
            self.client._worker.join(1)
        self.assertEqual(self.posts, [])
        self.assertTrue(any(item['operation'] and item['operation']['state'] == 'failed' for item in updates))

    def test_callback_outside_lock_and_close_cancels_inflight(self):
        self.client.refresh(); self.mode = 'pending'
        acquired = []
        def callback(snapshot):
            finished = threading.Event()
            def read():
                self.client.snapshot(); finished.set()
            thread = threading.Thread(target=read, daemon=True)
            thread.start()
            acquired.append(finished.wait(.3))
        self.client.on_update = callback
        operation = self.client.start_shutter('close-inflight')
        deadline = time.monotonic() + 1
        while operation.snapshot()['command_id'] is None and time.monotonic() < deadline:
            time.sleep(.002)
        self.client.close()
        self.assertEqual(self.done(operation)['state'], 'unknown')
        self.assertTrue(acquired and all(acquired))
        self.assertFalse(self.client.snapshot()['bridge_online'])
        self.assertFalse(self.done(self.client.command('connect'))['ok'])
        self.assertEqual(self.posts, ['shutter'])

    def test_dry_run_never_http_and_requires_manual_connection(self):
        client = CameraBridgeClient(Path(self.temp.name)/'dry.jsonl', dry_run=True)
        self.addCleanup(client.close)
        with patch.object(client, '_http', side_effect=AssertionError('HTTP forbidden')):
            self.assertFalse(self.done(client.start_shutter('before-connect'))['ok'])
            self.assertTrue(self.done(client.command('connect'))['ok'])
            self.assertTrue(self.done(client.start_shutter('first'))['ok'])
            self.assertFalse(self.done(client.start_shutter('first'))['ok'])
            self.assertTrue(self.done(client.command('disconnect'))['ok'])
            client.start(); client.refresh()
            self.assertFalse(client.snapshot()['connected'])


if __name__ == '__main__':
    unittest.main()
