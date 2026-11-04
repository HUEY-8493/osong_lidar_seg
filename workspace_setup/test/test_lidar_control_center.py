import ast
import importlib.util
import io
import json
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch, Mock
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location("control_center", Path(__file__).parents[1] / "scripts/lidar_control_center.py")
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


def pcd_archive(unsafe=False, bag=False):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        payloads = {"../escape": b"bad"} if unsafe else {}
        if bag:
            payloads.update({"metadata.yaml": b"rosbag2_bagfile_information:\n  message_count: 3\n", "sample.mcap": b"MCAP synthetic"})
        else:
            payloads["session.json"] = json.dumps({"schema_version": "2.2", "reason": "manual", "event_end_stamp_ns": 101000000000}).encode()
            header = "frame_index,stamp_sec,stamp_nanosec,raw_pcd_path,annotation_path,save_status\n"
            # The final quoted empty field is tested separately through an extra column.
            header = header.rstrip() + ",preview_path\n"
            rows = []
            for index, stamp in enumerate((100, 100, 101), 1):
                name = f"raw/{index:08d}.pcd"
                payloads[name] = f"full frame {index}".encode()
                rows.append(f'{index},{stamp},0,"{name}","","RAW_ONLY",\"\n')
            payloads["frames.csv"] = (header + "".join(rows)).encode()
            payloads["annotations/00000001.json"] = json.dumps({"objects": [{"track_id": 7, "lifecycle": "TRACKED_PREDICTED", "observed": False}]}).encode()
        for name, value in payloads.items():
            info = tarfile.TarInfo("./" + name if not name.startswith("..") else name)
            info.size = len(value)
            archive.addfile(info, io.BytesIO(value))
    return buffer.getvalue()


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.output = Path(self.temp.name)
        self.state = {"general": {"list": True}, "crane": {"list": False}}
        self.payload = pcd_archive()
        self.calls = []
        self.servers, self.threads = [], []
        case = self
        for role in ("general", "crane"):
            def handler_factory(name):
                class Handler(BaseHTTPRequestHandler):
                    def log_message(self, *args):
                        pass
                    def reply(self, value, status=200):
                        body = json.dumps(value).encode()
                        self.send_response(status)
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                    def do_GET(self):
                        case.calls.append((name, "GET", self.path))
                        state = case.state[name]
                        if self.path == "/api/lidar/status":
                            data = {"system_mode": state.get("mode", name), "bridge_connected": True,
                                    "lidar": {"pointcloud_active": state.get("healthy", True), "packet_active": True, "perception_active": state.get("perception_active", True)},
                                    "perception": {"vehicle_count": state.get("vehicle_count", 0)}}
                            if name == "general" and state.get("coordinator"):
                                data["crane_capture"] = {"manager_url": "http://existing:8765"}
                            self.reply(data)
                        elif self.path == "/api/lidar/capture":
                            self.reply({"captures": [state["job"]] if "job" in state else []}, 200 if state["list"] else 404)
                        elif self.path.endswith("/download"):
                            if not state.get("ready"):
                                self.reply({"error": "archiving"}, 409)
                                return
                            self.send_response(200)
                            self.send_header("Content-Length", str(len(case.payload) + (1 if state.get("short") else 0)))
                            self.end_headers()
                            self.wfile.write(case.payload)
                        elif self.path.startswith("/api/lidar/capture/"):
                            if "job" not in state:
                                self.reply({"error": "unknown"}, 404)
                            else:
                                self.reply(state["job"])
                        else:
                            self.reply({}, 404)
                    def do_POST(self):
                        case.calls.append((name, "POST", self.path))
                        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                        if self.path == "/api/lidar/capture":
                            case.state[name]["duration"] = body["duration_sec"]
                            case.state[name]["job"] = {"capture_id": "20261014_120000", "state": "complete",
                                                        "archive_path": "/capture/test.tar.gz" if case.state[name].get("ready") else None,
                                                        "size_bytes": len(case.payload) if case.state[name].get("ready") else None}
                            self.reply({"success": True, "capture_id": "20261014_120000"}, 202)
                        else:
                            self.reply({"success": not case.state[name].get("command_failed", False)})
                return Handler
            server = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(role))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.servers.append(server)
            self.threads.append(thread)
        self.client = m.ControlCenter(*(f"http://127.0.0.1:{s.server_port}" for s in self.servers), output=self.output)
        self.general, self.crane = self.client.targets.values()
        self.client.poll_target(self.general)
        self.client.poll_target(self.crane)
    def tearDown(self):
        self.client.close()
        for server, thread in zip(self.servers, self.threads):
            server.shutdown()
            server.server_close()
            thread.join()
        self.temp.cleanup()
    def ready_job(self, target="crane"):
        state = self.state[target]
        state["ready"] = True
        state["job"].update(archive_path="/capture/test.tar.gz", size_bytes=len(self.payload))
    def test_legacy_capture_archive_readiness_download_and_1hz(self):
        self.assertFalse(self.crane.list_supported)
        self.client.start_capture(self.crane, 60)
        self.client.capture_step(self.crane)
        self.assertFalse(any(path.endswith('/download') for _, _, path in self.calls))
        self.ready_job()
        self.client.capture_step(self.crane)
        folder = self.output / "crane/captures/20261014_120000"
        self.assertEqual((folder / "original.tar.gz").read_bytes(), self.payload)
        with tarfile.open(folder / "raw_1hz.tar.gz") as archive:
            self.assertEqual([n for n in archive.getnames() if n.startswith("./raw/")], ["./raw/00000001.pcd", "./raw/00000003.pcd"])
        self.assertEqual(json.loads((folder / "summary.json").read_text())["frames"], 3)
        self.assertEqual(self.crane.capture_id, "")
        self.assertFalse((self.output / "crane/pending.json").exists())
        before = len(self.calls)
        self.client.poll_target(self.crane)
        self.assertFalse(any(path == '/api/lidar/capture' for _, method, path in self.calls[before:] if method == 'GET'))
    def test_existing_general_coordinator_blocks_new_crane_request(self):
        self.state['general']['coordinator'] = True
        self.client.poll_target(self.general)
        with self.assertRaisesRegex(RuntimeError, '기존 Crane 수집기'):
            self.client.start_capture(self.crane, 60)
        self.assertFalse(any(method == 'POST' for _, method, _ in self.calls))
    def test_ambiguous_post_is_checkpointed_and_not_repeated(self):
        real = m.urllib.request.urlopen
        def lost(request, **kwargs):
            if isinstance(request, m.urllib.request.Request) and request.get_method() == 'POST':
                with real(request, **kwargs) as response:
                    response.read()
                raise m.urllib.error.URLError('response lost')
            return real(request, **kwargs)
        with patch.object(m.urllib.request, 'urlopen', side_effect=lost):
            with self.assertRaises(m.urllib.error.URLError):
                self.client.start_capture(self.crane, 60)
        restored = m.ControlCenter(self.general.url, self.crane.url, self.output)
        self.assertTrue(restored.targets['crane'].uncertain)
        restored.capture_step(restored.targets['crane'])
        self.assertEqual(sum(method == 'POST' for _, method, _ in self.calls), 1)
    def test_known_id_recovers_after_restart_and_short_download_is_not_complete(self):
        self.client.start_capture(self.crane, 60)
        restored = m.ControlCenter(self.general.url, self.crane.url, self.output)
        target = restored.targets['crane']
        self.assertEqual(target.capture_id, self.crane.capture_id)
        self.ready_job()
        self.state['crane']['short'] = True
        with self.assertRaises(Exception):
            restored.capture_step(target)
        folder = self.output / 'crane/captures/20261014_120000'
        self.assertFalse((folder / 'download.json').exists())
        self.assertTrue(target.capture_id)
        self.state['crane']['short'] = False
        restored.capture_step(target)
        self.assertTrue((folder / 'download.json').exists())
    def test_unsafe_archive_is_not_extracted(self):
        path = self.output / 'bad.tar.gz'
        path.write_bytes(pcd_archive(unsafe=True))
        with self.assertRaisesRegex(RuntimeError, '안전하지 않은'):
            self.client.inspect_archive(path, self.output, self.crane)
        self.assertFalse((self.output.parent / 'escape').exists())
    def test_general_completed_capture_collection_preserves_frames(self):
        self.state['general']['ready'] = True
        self.client.start_capture(self.general, 10)
        self.client.capture_step(self.general)
        self.assertEqual(self.state['general']['duration'], 10)
        folder = self.output / 'general/captures/20261014_120000'
        self.assertTrue((folder / 'original.tar.gz').exists())
        self.assertFalse((folder / 'raw_1hz.tar.gz').exists())
    def test_stop_repeat_is_immediate_when_offline_and_no_stale_commands(self):
        self.crane.repeat = True
        self.crane.connected = False
        self.client.submit('crane', 'repeat', False)
        self.assertFalse(self.crane.repeat)
        self.client.submit('crane', 'normal')
        self.assertTrue(self.crane.actions.empty())
    def test_command_mode_guard_and_server_failure(self):
        self.crane.status['system_mode'] = 'general'
        with self.assertRaises(RuntimeError):
            self.client.action(self.crane, 'standby')
        self.crane.status['system_mode'] = 'crane'
        self.state['crane']['command_failed'] = True
        with self.assertRaisesRegex(RuntimeError, '명령 실패'):
            self.client.action(self.crane, 'normal')
    def test_three_hour_alert_once_reset_and_no_implicit_ssh_recording(self):
        self.state['general']['vehicle_count'] = 1
        self.general.operated_since = time.monotonic() - 10801
        self.general.ssh = 'user@host'
        self.client.poll_target(self.general)
        self.client.poll_target(self.general)
        self.assertEqual(sum(e['category'] == 'ALERT' for e in self.client.events), 1)
        self.assertTrue(self.general.actions.empty())
        self.state['general']['healthy'] = False
        self.client.poll_target(self.general)
        self.assertFalse(self.general.alert_sent)
        self.assertIsNone(self.general.operated_since)
    def test_no_vehicle_or_stale_perception_never_triggers_three_hours(self):
        for count, fresh in ((0, True), (2, False)):
            self.state['general']['vehicle_count'] = count
            self.state['general']['perception_active'] = fresh
            self.general.operated_since = time.monotonic() - 10801
            self.client.poll_target(self.general)
            self.assertIsNone(self.general.operated_since)
            self.assertFalse(self.general.alert_sent)
        self.assertFalse(any(e['category'] == 'ALERT' for e in self.client.events))

    def test_manager_vehicle_timer_requires_recognition_and_resets(self):
        tree = ast.parse((Path(__file__).parents[1] / 'scripts/guardian_manager.py').read_text())
        condition = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
                         and isinstance(node.test, ast.Compare)
                         and ast.unparse(node.test) == "_runtime_mode_config.mode == 'general'"
                         and any(isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute)
                                 and child.func.attr == 'start_rosbag' for child in ast.walk(node)))
        code = compile(ast.Module(body=[condition], type_ignores=[]), '<manager vehicle timer>', 'exec')
        manager = SimpleNamespace(_general_started_at=None, _general_alert=None,
                                  start_rosbag=Mock(return_value='bag_test'))
        bridge = {'lidar': {'pointcloud_active': True, 'perception_active': True},
                  'perception': {'vehicle_count': 0}}
        env = {'_runtime_mode_config': SimpleNamespace(mode='general'),
               '_capture_manager': manager, '_bridge_state': SimpleNamespace(snapshot=lambda _: (bridge, True)),
               'BRIDGE_HEALTH_TIMEOUT_SEC': 2, '_packet_monitor': SimpleNamespace(health=lambda: (True, 'ok')),
               'requested_state': 'normal', 'GENERAL_ALERT_AFTER_SEC': 10800,
               'GENERAL_AUTO_CAPTURE_ENABLED': False, 'time': time, 'log': Mock(), 'now': 0}
        def tick(now):
            env['now'] = now
            exec(code, env)
        tick(0); tick(20000)
        self.assertIsNone(manager._general_started_at)
        manager.start_rosbag.assert_not_called()
        bridge['perception']['vehicle_count'] = 1
        tick(20000); tick(30799)
        manager.start_rosbag.assert_not_called()
        tick(30800); tick(30801)
        manager.start_rosbag.assert_not_called()
        self.assertIsNone(manager._general_alert['capture_id'])
        bridge['perception']['vehicle_count'] = 0
        tick(30802)
        self.assertIsNone(manager._general_started_at)
        bridge['perception']['vehicle_count'] = 1
        tick(30803)
        self.assertEqual(manager._general_started_at, 30803)
        self.assertIsNone(manager._general_alert)
        bridge['lidar']['perception_active'] = False
        tick(50000)
        self.assertIsNone(manager._general_started_at)
        bridge['lidar']['perception_active'] = True
        env['GENERAL_AUTO_CAPTURE_ENABLED'] = True
        tick(50001); tick(60801); tick(60802)
        manager.start_rosbag.assert_called_once()

    def test_ssh_journal_and_existing_bag_owner_guard(self):
        self.general.ssh = 'user@host'
        with patch.object(m.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, b'journal entry\n', b'')) as run:
            self.client.ssh_collect(self.general, 'journal')
            self.assertIn('journalctl', run.call_args.args[0][-1])
        self.assertTrue(list((self.output / 'general/remote_logs').glob('*.log')))
        self.general.status['operation_elapsed_sec'] = 0
        with self.assertRaisesRegex(RuntimeError, '중복'):
            self.client.action(self.general, 'bag')
    def test_ssh_track_snapshot_without_dds_or_server_updates(self):
        self.general.ssh = 'user@host'
        response = b"objects:\n- id: 91\n  class_name: VEHICLE\n  lifecycle_state: TRACKED_PREDICTED\n  decision_reason: TEST_REASON\n  lost_frames: 2\n---\n"
        with patch.object(m.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, response, b'')) as run:
            self.client.ssh_collect(self.general, 'tracks')
            self.assertIn('topic echo', run.call_args.args[0][-1])
            self.assertIn('/tracked_objects', run.call_args.args[0][-1])
        row = self.client.tracks['general:SSH /tracked_objects'][0]
        self.assertEqual(row['lifecycle_state'], 'TRACKED_PREDICTED')
        self.assertEqual(row['decision_reason'], 'TEST_REASON')
        self.assertIn('스냅샷', self.client.track_source)
        self.assertTrue(list((self.output / 'general/tracking_snapshots').glob('*.json')))

    def test_legacy_general_auto_sessions_via_existing_ssh_files(self):
        self.general.ssh = 'user@host'
        calls = []
        def ssh(args, **kwargs):
            calls.append(args[-1])
            if 'python3 -c' in args[-1]:
                return subprocess.CompletedProcess(args, 0, b'["recording_test_0001"]', b'')
            kwargs['stdout'].write(self.payload)
            return subprocess.CompletedProcess(args, 0, b'', b'')
        with patch.object(m.subprocess, 'run', side_effect=ssh):
            self.client.collect_sessions(self.general)
            self.client.collect_sessions(self.general)
        folder = self.output / 'general/captures/ssh_recording_test_0001'
        self.assertTrue((folder / 'download.json').exists())
        self.assertEqual(self.general.captures['ssh_recording_test_0001']['kind'], 'ssh_session')
        self.assertEqual(sum('tar -C' in call for call in calls), 1)
        restored = m.ControlCenter(self.general.url, self.crane.url, self.output)
        self.assertIn('ssh_recording_test_0001', restored.targets['general'].captures)

    def test_bag_archive_inspection(self):
        path = self.output / 'bag.tar.gz'
        path.write_bytes(pcd_archive(bag=True))
        self.client.inspect_archive(path, self.output, self.general)
        self.assertEqual(json.loads((self.output / 'summary.json').read_text())['kind'], 'rosbag')


class TerminalTests(unittest.TestCase):
    def setUp(self):
        import curses
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.client = m.ControlCenter('http://127.0.0.1:1', 'http://127.0.0.1:2', self.temp.name, demo=True)
        self.addCleanup(self.client.close)
        self.screen = Mock()
        self.screen.getmaxyx.return_value = (30, 120)
        colors = patch.object(curses, 'color_pair', return_value=0)
        colors.start()
        self.addCleanup(colors.stop)
        self.ui = m.Terminal(self.screen, self.client)
        for target in self.client.targets.values():
            self.client.demo_status(target)

    def test_terminal_commands_confirm_target_and_immediate_repeat_stop(self):
        self.ui.key('c')
        self.screen.get_wch.side_effect = ['\x1b']
        self.ui.key('r')
        self.assertTrue(self.client.targets['crane'].actions.empty())
        self.screen.get_wch.side_effect = ['y', '\n']
        self.ui.key('r')
        action, duration, _ = self.client.targets['crane'].actions.get_nowait()
        self.assertEqual((action, duration), ('capture', 60))
        self.ui.key('\t')
        self.screen.get_wch.side_effect = ['y', '\n']
        self.ui.key('r')
        action, duration, _ = self.client.targets['general'].actions.get_nowait()
        self.assertEqual((action, duration), ('capture', 10))
        self.client.targets['crane'].repeat = True
        self.screen.get_wch.side_effect = AssertionError('Stopping repeat must not prompt')
        self.ui.key('b')
        self.assertFalse(self.client.targets['crane'].repeat)

    def test_terminal_tracks_details_logs_and_resize(self):
        self.ui.key('2')
        self.ui.render()
        self.assertEqual(len(self.ui.rows), 2)
        self.ui.key(self.ui.curses.KEY_DOWN)
        self.ui.key('\n')
        self.assertIn('TRACKED_PREDICTED', self.ui.details)
        self.assertIn('BOUNDED_PREDICTION', self.ui.details)
        self.ui.key('\x1b')
        self.client.log(self.client.targets['general'], 'ERROR', 'test failure', response={'detail': 'full reason'})
        self.ui.key('4')
        self.ui.render()
        self.ui.key('\n')
        self.assertIn('full reason', self.ui.details)
        self.ui.key('\x1b')
        self.screen.getmaxyx.return_value = (10, 20)
        self.ui.render()
        self.assertFalse(self.ui.key('q'))

    def test_terminal_unicode_clipping_and_missing_tk(self):
        with patch.dict(sys.modules, {'tkinter': None}):
            self.ui.render()
        self.screen.getmaxyx.return_value = (18, 7)
        self.ui.write(2, '차량인지ABCDE')
        self.assertEqual(self.screen.addstr.call_args.args[2], '차량인')


if __name__ == '__main__':
    unittest.main()
