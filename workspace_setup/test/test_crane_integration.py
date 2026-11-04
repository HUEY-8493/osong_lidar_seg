"""Crane deployment contract tests; no hardware or operational Redis access."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("crane_manager_contract", SCRIPTS / "guardian_manager.py")
manager = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = manager
spec.loader.exec_module(manager)


class CraneManagerContractTest(unittest.TestCase):
    def setUp(self):
        self.saved = (manager._runtime_mode_config, manager._bridge_state,
                      manager._crane_state, manager._crane_state_from_bridge)
        manager._runtime_mode_config = manager.MODE_CONFIG["crane"]
        manager._bridge_state = manager.BridgeState()
        manager._crane_state = manager.MobileCraneState()
        manager._crane_state.update_crane_state(1, 3.09, 1, 1234)

    def tearDown(self):
        (manager._runtime_mode_config, manager._bridge_state,
         manager._crane_state, manager._crane_state_from_bridge) = self.saved

    def receive(self, cloud=True, perception=True):
        manager._crane_state_from_bridge = True
        manager._bridge_state.update({"mode": "crane", "perception": {},
                                     "lidar": {"pointcloud_active": cloud, "perception_active": perception}}, "crane")

    def test_sensor_timestamp_is_preserved_for_delayed_publication(self):
        state = manager.MobileCraneState()
        self.assertEqual(state.build_payload()["timestamp"], 0)
        for stamp in (1792821834326, 1792821834426, 0):
            state.update_crane_state("SUSPENDED", 2.0, 0, stamp)
            with patch("mobile_crane_status.time.time", return_value=9999999999):
                self.assertEqual(state.build_payload()["timestamp"], stamp)
                self.assertEqual(state.build_payload()["timestamp"], stamp)
        for bad in (-1, 1.2, True, "123"):
            with self.assertRaises(ValueError):
                state.update_crane_state("SUSPENDED", 2.0, 0, bad)
        state.update_crane_state("IDLE", 0.0, 0, None)
        self.assertEqual(state.build_payload()["timestamp"], 0)

    def test_crane_input_loss_retains_last_work_and_sensor_timestamp(self):
        self.receive()
        payload = manager.build_external_payload()
        self.assertEqual(set(payload), {"working_status", "object_length", "crane_moving", "lidar_status", "lidar_message", "timestamp"})
        self.assertEqual((payload["working_status"], payload["crane_moving"]), ("SUSPENDED", 1))
        for cloud, perception in ((False, True), (True, False), (False, False)):
            self.receive(cloud, perception)
            payload = manager.build_external_payload()
            self.assertEqual((payload["working_status"], payload["object_length"], payload["crane_moving"]), ("SUSPENDED", 3.09, 1))
            self.assertEqual(payload["timestamp"], 1234)
        self.receive()
        manager._bridge_state.received_at = time.monotonic() - manager.BRIDGE_HEALTH_TIMEOUT_SEC - .1
        self.assertEqual(manager.build_external_payload()["working_status"], "SUSPENDED")
        self.receive()
        manager._crane_state.update_crane_state(1, 2.5, 0, 5678)
        self.assertEqual(manager.build_external_payload()["object_length"], 2.5)
        self.receive(False, False)
        manager.update_crane_state(1, 4.0, 0, 6789)
        self.assertEqual(manager.build_external_payload()["object_length"], 4.0)
        self.receive(False, True)
        manager._bridge_state.payload["perception"]["control_mode"] = "MANUAL_OVERRIDE"
        self.assertEqual(manager.build_external_payload()["working_status"], "SUSPENDED")

    def test_health_only_and_disconnected_bridge_retain_detailed_crane_state(self):
        manager._bridge_state.update({"mode": "crane",
            "lidar": {"pointcloud_active": True, "perception_active": True},
            "perception": {"state": "LOWERING", "sensor_timestamp_ms": 1234}}, "crane")
        manager._bridge_state.update({"mode": "crane",
            "lidar": {"pointcloud_active": False, "perception_active": False},
            "perception": {}}, "crane")
        manager._bridge_state.received_at = time.monotonic() - manager.BRIDGE_HEALTH_TIMEOUT_SEC - .1
        cached, fresh = manager._bridge_state.snapshot(manager.BRIDGE_HEALTH_TIMEOUT_SEC)
        self.assertFalse(fresh)
        self.assertEqual(cached["perception"]["state"], "LOWERING")
        self.assertEqual(cached["perception"]["sensor_timestamp_ms"], 1234)
        manager._bridge_state.update({"mode": "crane",
            "lidar": {"pointcloud_active": True, "perception_active": True},
            "perception": {"state": "SUSPENDED", "sensor_timestamp_ms": 5678}}, "crane")
        cached, fresh = manager._bridge_state.snapshot(manager.BRIDGE_HEALTH_TIMEOUT_SEC)
        self.assertTrue(fresh)
        self.assertEqual(cached["perception"]["state"], "SUSPENDED")

    def test_legacy_endpoint_and_general_profile_are_unchanged(self):
        self.assertEqual(manager.build_external_payload()["working_status"], "SUSPENDED")
        manager._runtime_mode_config = manager.MODE_CONFIG["general"]
        self.receive(False, False)
        self.assertEqual(manager.build_external_payload(), manager._vehicle_state.build_payload())
        manager._vehicle_state.update_vehicle_state(0, "", 0.0, 1234, 0.0)
        self.assertEqual(manager.build_external_payload()["vehicle_status"], "")

    def test_string_work_contract_and_invalid_values(self):
        expected={"working_status","object_length","crane_moving","lidar_status","lidar_message","timestamp"}
        for value,label in [(0,"IDLE"),(1,"SUSPENDED"),(2,"LOWERING_CONFIRM"),(3,"LIFTING"),("LIFTING","LIFTING"),("IDLE","IDLE"),("SUSPENDED","SUSPENDED"),("LOWERING_CONFIRM","LOWERING_CONFIRM"),("LOWERING","LOWERING"),("SUSPENDED","SUSPENDED")]:
            manager.update_crane_state(value,3.09,1,1234)
            payload=manager.build_external_payload()
            self.assertEqual(set(payload),expected)
            self.assertEqual(payload["working_status"],label)
            self.assertIs(type(payload["object_length"]),float)
            self.assertIs(type(payload["crane_moving"]),int)
        for bad in [True,False,4,-1,"UNKNOWN","1","",None]:
            with self.assertRaises(ValueError):
                manager.update_crane_state(bad,3.09,1,1234)

    def test_five_fsm_states_reach_manager_payload_through_bridge_handler(self):
        from types import SimpleNamespace
        projection_path = SCRIPTS.parents[1] / "guardian_lidar_bridge/guardian_lidar_bridge/state_projection.py"
        projection_spec = importlib.util.spec_from_file_location("five_state_projection", projection_path)
        projection = importlib.util.module_from_spec(projection_spec)
        projection_spec.loader.exec_module(projection)
        for state in ("IDLE", "LIFTING", "SUSPENDED", "LOWERING", "LOWERING_CONFIRM",
                      "COMPLETE_CONFIRM", "LOWERING_CONFIRMATION"):
            expected = "LOWERING_CONFIRM" if state in ("COMPLETE_CONFIRM", "LOWERING_CONFIRMATION") else state
            status = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=0)),
                state=state, working_status="working", crane_moving=False, beam_length=2.0,
                beam_cluster_valid=True, work_complete=expected=="LOWERING_CONFIRM", motion_dz=0.0)
            perception = projection.crane_summary(status)
            body = {"mode": "crane", "perception": perception,
                    "lidar": {"pointcloud_active": True, "perception_active": True}}
            encoded = json.dumps(body).encode()
            handler = object.__new__(manager._json_handler(None))
            handler.path = "/api/internal/bridge"
            handler.headers = {"Content-Length": str(len(encoded))}
            handler.rfile = manager.io.BytesIO(encoded)
            response = []
            handler._respond = lambda code, data: response.append(code)
            with patch.object(manager, "GUARDIAN_BRIDGE_TOKEN", ""):
                handler.do_POST()
            self.assertEqual(response, [200])
            payload = manager.build_external_payload()
            self.assertEqual(payload["working_status"], expected)
            self.assertEqual(payload["timestamp"], 12000)
            self.receive(False, False)
            self.assertEqual(manager.build_external_payload()["working_status"], expected)

    def test_crane_docker_launch_uses_crane_tf_and_overlay_only_in_crane(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            docker = folder / "docker"
            docker.write_text("#!/usr/bin/env python3\nimport sys,json,os\nif sys.argv[1]=='ps': print('ros2_jazzy_lidar')\nelif sys.argv[1]=='exec': open(os.environ['TEST_DOCKER_CAPTURE'],'w').write(json.dumps(sys.argv[2:]))\n")
            docker.chmod(0o755)
            # Execute the captured container shell with a mocked ros2. Its
            # fixed source paths may be absent here; the final launch args are captured.
            ros = folder / "ros2"
            ros.write_text("#!/usr/bin/env python3\nimport sys,json,os\nopen(os.environ['TEST_ROS_CAPTURE'],'w').write(json.dumps(sys.argv[1:]))\n")
            ros.chmod(0o755)
            for mode in ("crane", "general"):
                env = dict(os.environ, PATH=str(folder) + os.pathsep + os.environ["PATH"],
                           GUARDIAN_BRIDGE_MODE=mode, GUARDIAN_PERCEPTION_PARAMS_FILE="/root/ros2_ws/install/cx128s2_bev_perception/share/cx128s2_bev_perception/config/cx128s2_bev_perception_" + mode + ".yaml",
                           TEST_DOCKER_CAPTURE=str(folder / "docker.json"), TEST_ROS_CAPTURE=str(folder / "ros.json"))
                subprocess.run(["bash", str(SCRIPTS / "run_lidar.sh")], env=env, check=True, capture_output=True)
                args = json.loads((folder / "docker.json").read_text())
                shell = args[-1].replace("source /opt/ros/jazzy/setup.bash", ":").replace("source /root/ros2_ws/install/setup.bash", ":")
                container_env = dict(env)
                for index, argument in enumerate(args[:-1]):
                    if argument == "-e":
                        key, value = args[index + 1].split("=", 1)
                        container_env[key] = value
                subprocess.run(["bash", "-c", shell], env=container_env, check=True, capture_output=True)
                actual = json.loads((folder / "ros.json").read_text())
                self.assertEqual(actual[:3], ["launch", "lidar_bringup", "lidar_bringup.launch.py"])
                self.assertIn("system_mode:=" + mode, actual)
                self.assertIn("guardian_bridge_mode:=" + mode, actual)



class GeneralExitContractTest(unittest.TestCase):
    def setUp(self):
        self.saved = (manager._runtime_mode_config, manager._bridge_state, manager._vehicle_state)
        manager._runtime_mode_config = manager.MODE_CONFIG["general"]
        manager._bridge_state = manager.BridgeState()
        manager._vehicle_state = manager.GeneralVehicleState()

    def tearDown(self):
        manager._runtime_mode_config, manager._bridge_state, manager._vehicle_state = self.saved

    def receive(self, exist, status, cloud=True, perception=True):
        # Execute the actual POST handler without opening a socket or contacting Redis.
        body = {"mode": "general", "lidar": {"pointcloud_active": cloud,
                "perception_active": perception}, "perception": {
                "vehicle_exist": exist, "vehicle_status": status, "vehicle_count": exist,
                "vehicle_distance": 3.0 if exist else 0.0, "vehicle_length": 4.0 if exist else 0.0,
                "sensor_timestamp_ms": 1234}}
        encoded = json.dumps(body).encode()
        handler = object.__new__(manager._json_handler(None))
        handler.path = "/api/internal/bridge"
        handler.headers = {"Content-Length": str(len(encoded))}
        handler.rfile = manager.io.BytesIO(encoded)
        response = []
        handler._respond = lambda code, data: response.append(code)
        with patch.object(manager, "GUARDIAN_BRIDGE_TOKEN", ""):
            handler.do_POST()
        self.assertEqual(response, [200])

    def assert_vehicle(self, exist, status):
        payload = manager.build_external_payload()
        self.assertEqual((payload["vehicle_exist"], payload["vehicle_status"]), (exist, status))
        if not exist:
            self.assertEqual((payload["vehicle_distance"], payload["vehicle_length"]), (0.0, 0.0))

    def test_exit_absence_and_new_vehicle_via_real_handler(self):
        for exist, status in ((0, ""), (1, "stopping"), (1, "exiting"),
                              (1, "exiting"), (0, ""), (1, "stopping")):
            # Temporary loss already allowed by perception remains a public vehicle.
            self.receive(exist, status)
            self.assert_vehicle(exist, status)

    def test_input_loss_and_bridge_disconnect_mask_stale_exit(self):
        self.receive(1, "exiting")
        self.assert_vehicle(1, "exiting")
        for cloud, perception in ((False, True), (True, False), (False, False)):
            self.receive(1, "exiting", cloud, perception)
            self.assert_vehicle(0, "")
            bridge, fresh = manager._bridge_state.snapshot(manager.BRIDGE_HEALTH_TIMEOUT_SEC)
            self.assertTrue(fresh)
            self.assertEqual(bridge["perception"]["vehicle_count"], 0)
            # Neither raw transport cache nor per-track state is deleted by masking.
            self.assertEqual(manager._bridge_state.payload["perception"]["vehicle_status"], "exiting")
        self.receive(1, "exiting")
        manager._bridge_state.received_at -= manager.BRIDGE_HEALTH_TIMEOUT_SEC + 1
        self.assert_vehicle(0, "")
        self.receive(1, "stopping")
        self.assert_vehicle(1, "stopping")

    def test_manager_display_and_redis_agree_after_timeout(self):
        self.receive(1, "exiting", False, False)
        controller = type("Controller", (), {"status": lambda self: {}})()
        with patch.object(manager, "get_lidar_runtime_status", return_value={}), \
                patch.object(manager, "_packet_monitor", None):
            for disconnected in (False, True):
                if disconnected:
                    manager._bridge_state.received_at -= manager.BRIDGE_HEALTH_TIMEOUT_SEC + 1
                summary = manager.build_status_payload(controller)["perception"]
                self.assertEqual((summary["vehicle_count"], summary["vehicle_exist"],
                                  summary["vehicle_status"]), (0, 0, ""))
                self.assert_vehicle(0, "")




class UnifiedModeContractTest(unittest.TestCase):
    def test_conflicting_script_modes_do_not_start_docker(self):
        env = dict(os.environ, GUARDIAN_SYSTEM_MODE="crane", GUARDIAN_BRIDGE_MODE="general")
        result = subprocess.run(["bash", str(SCRIPTS / "run_lidar.sh")], env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("modes disagree", result.stderr)

    def test_common_launch_resolves_both_profiles_and_rejects_mixed_bridge(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        root = SCRIPTS.parents[1]
        path = root / "lidar_bringup/launch/lidar_bringup.launch.py"
        spec = importlib.util.spec_from_file_location("mode_bringup_contract", path)
        launch_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch_module)
        def share(name):
            return str(root / name)
        with patch.object(launch_module, "get_package_share_directory", side_effect=share):
            for mode in ("crane", "general"):
                context = LaunchContext()
                context.launch_configurations.update(system_mode=mode, use_lidar_driver="false")
                for action in launch_module.generate_launch_description().entities:
                    if isinstance(action, DeclareLaunchArgument):
                        action.execute(context)
                values = context.launch_configurations
                self.assertEqual(values["guardian_bridge_mode"], mode)
                self.assertTrue(values["tf_config"].endswith("lidar_tf_crane.yaml" if mode == "crane" else "lidar_tf.yaml"))
                self.assertTrue(values["rviz_config"].endswith("lidar_crane_work.rviz" if mode == "crane" else "lidar.rviz"))
                self.assertEqual(bool(values["perception_params_overlay_file"]), mode == "crane")
                if mode == "crane":
                    self.assertTrue(values["perception_params_file"].endswith("cx128s2_bev_perception_crane.yaml"))
                else:
                    self.assertEqual(values["perception_params_file"], "")
                actions = launch_module.launch_setup(context)
                self.assertTrue(actions)
                values["guardian_bridge_mode"] = "general" if mode == "crane" else "crane"
                with self.assertRaises(ValueError):
                    launch_module.launch_setup(context)
                values["guardian_bridge_mode"] = mode
                values["rviz_config"] = "/nonexistent/mode_contract.rviz"
                with self.assertRaises(FileNotFoundError):
                    launch_module.create_rviz_action(context, False)

    def test_manager_passes_common_mode_and_explicit_rviz_preset(self):
        saved = manager._runtime_mode_config
        try:
            for mode in ("crane", "general"):
                manager._runtime_mode_config = manager.MODE_CONFIG[mode]
                done = subprocess.CompletedProcess([], 0, "", "")
                with patch.object(manager, "_ros_process_running", return_value=False), \
                     patch.object(manager.os.path, "isfile", return_value=True), \
                     patch.object(manager.subprocess, "run", return_value=done) as run:
                    self.assertTrue(manager.start_lidar_docker())
                    env = run.call_args.kwargs["env"]
                    self.assertEqual(env["GUARDIAN_SYSTEM_MODE"], mode)
                    self.assertEqual(env["GUARDIAN_BRIDGE_MODE"], mode)
                with patch.object(manager, "RVIZ_AUTO_START", True), \
                     patch.object(manager, "_ros_process_running", return_value=False), \
                     patch.object(manager, "_container_running", return_value=True), \
                     patch.object(manager.subprocess, "run", return_value=done) as run:
                    self.assertTrue(manager.ensure_rviz2_running())
                    command = run.call_args.args[0][-1]
                    self.assertIn("exec rviz2 -d ", command)
                    self.assertIn("lidar_crane_work.rviz" if mode == "crane" else "lidar.rviz", command)
        finally:
            manager._runtime_mode_config = saved

    def test_existing_installer_updates_mode_preserves_unit_and_custom_wrapper(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            source = folder / "scripts"
            source.mkdir()
            for name in ("guardian_manager.py", "mobile_crane_status.py", "cx128s2_control.py", "run_lidar.sh"):
                (source / name).write_text((SCRIPTS / name).read_text())
            runtime = folder / "runtime"
            runtime.mkdir()
            unit = folder / "guardian-manager.service"
            unit.write_text("existing service with user options\n")
            wrapper = runtime / "start_guardian_manager.sh"
            original = '#!/bin/bash\nexport DISPLAY=:7\n# local custom setting\nexec /usr/bin/python3 "$MANAGER_PATH"\n'
            wrapper.write_text(original)
            installer = (SCRIPTS / "setup_guardian_manager_service.sh").read_text()
            installer = installer.replace('SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"', f'SERVICE_PATH="{unit}"')
            installer = installer.replace('WORK_DIR="${TARGET_HOME}/docker/ros2_jazzy"', f'WORK_DIR="{runtime}"')
            script = source / "setup_guardian_manager_service.sh"
            script.write_text(installer)
            tools = folder / "bin"
            tools.mkdir()
            fake_id = tools / "id"
            fake_id.write_text('#!/bin/bash\nif [ "$#" = 1 ] && [ "$1" = -u ]; then echo 0; else /usr/bin/id "$@"; fi\n')
            fake_id.chmod(0o755)
            fake_systemctl = tools / "systemctl"
            fake_systemctl.write_text('#!/bin/bash\necho "$*" >> "$TEST_SERVICE_CALLS"\n')
            fake_systemctl.chmod(0o755)
            env = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ["PATH"],
                       SUDO_USER=__import__("pwd").getpwuid(os.getuid()).pw_name,
                       TEST_SERVICE_CALLS=str(folder / "service_calls"))
            for mode in ("crane", "general", "crane"):
                result = subprocess.run(["bash", str(script), mode], env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f'"$MANAGER_PATH" "{mode}"', wrapper.read_text())
                self.assertIn('export DISPLAY=:7\n# local custom setting\n', wrapper.read_text())
                self.assertEqual(unit.read_text(), "existing service with user options\n")
            before = wrapper.read_text()
            subprocess.run(["bash", str(script)], env=env, check=True, capture_output=True)
            self.assertEqual(wrapper.read_text(), before)
            self.assertEqual(wrapper.with_name(wrapper.name + ".mode-backup").read_text(), original)
            calls = (folder / "service_calls").read_text()
            self.assertNotIn("disable", calls)
            self.assertNotIn("enable", calls)
            self.assertEqual(calls.count("restart guardian-manager.service"), 3)


if __name__ == "__main__":
    unittest.main()
