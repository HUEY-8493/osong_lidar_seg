import unittest
from types import SimpleNamespace

from guardian_lidar_bridge.state_projection import FirstVehicleSelector, crane_summary, general_summary


def vehicle(track_id, age, distance, status="entering", length=4.25):
    return SimpleNamespace(id=track_id, age=age, distance=distance, vehicle_status=status,
                           dimensions=SimpleNamespace(x=length))


class FirstVehicleSelectorTest(unittest.TestCase):
    def test_longest_vehicle_replaces_selected_track_and_supplies_its_distance(self):
        selector = FirstVehicleSelector()
        far = vehicle(10, 20, 30., "stopped", length=4.)
        self.assertEqual(selector.select([far]).id, 10)
        longest = vehicle(11, 1, 3., "entering", length=12.)
        selected = selector.select([far, longest])
        self.assertEqual(selected.id, 11)
        payload = general_summary([], [], [], [far, longest], 1234, selected)
        self.assertEqual((payload["vehicle_length"], payload["vehicle_distance"],
                          payload["vehicle_status"]), (12., 3., "entering"))
        far.dimensions.x = 13.
        self.assertEqual(selector.select([longest, far]).id, 10)

    def test_equal_lengths_keep_selection_and_disappearance_reselects_longest(self):
        selector = FirstVehicleSelector()
        selected = vehicle(20, 1, 2., length=8.)
        selector.select([selected])
        older = vehicle(10, 50, 40., length=8.)
        self.assertEqual(selector.select([older, selected]).id, 20)
        shorter = vehicle(30, 99, 50., length=5.)
        self.assertEqual(selector.select([shorter, older]).id, 10)

    def test_invalid_lengths_cannot_win_and_empty_input_clears_selection(self):
        selector = FirstVehicleSelector()
        good = vehicle(1, 1, 2., length=6.)
        bad = [vehicle(i+2, 100, 20., length=n)
               for i, n in enumerate([float('nan'), float('inf'), -1., 0.])]
        self.assertEqual(selector.select(bad + [good]).id, 1)
        self.assertIsNone(selector.select(bad))
        self.assertIsNone(selector.selected_id)

    def test_unavailable_input_masks_output_without_losing_selection(self):
        selector = FirstVehicleSelector()
        exiting = vehicle(10, 7, 3.0, "exiting")
        selected = selector.select([exiting])
        empty = general_summary([exiting], [exiting], [], [exiting], 1234,
                                selected, current=False)
        self.assertEqual((empty["vehicle_exist"], empty["vehicle_status"],
                          empty["vehicle_count"]), (0, "", 0))
        self.assertEqual((empty["object_count"], empty["tracked_object_count"]), (0, 0))
        self.assertEqual(selector.selected_id, 10)
        self.assertEqual(general_summary([], [], [], [exiting], 1234,
                                        selected)["vehicle_status"], "exiting")
        self.assertIsNone(selector.select([]))
        arriving = vehicle(11, 1, 10.0, "stopping")
        self.assertEqual(selector.select([arriving]).vehicle_status, "stopping")

    def test_absent_vehicle_and_missing_motion_are_not_stopped(self):
        empty = general_summary([], [], [], [], 1234, None)
        self.assertEqual(empty["vehicle_exist"], 0)
        self.assertEqual(empty["vehicle_status"], "")
        for status in ("", "unknown"):
            selected = vehicle(10, 1, 9.0, status)
            summary = general_summary([], [], [], [selected], 1234, selected)
            self.assertEqual(summary["vehicle_status"], "stopping")

    def test_keeps_first_track_when_another_vehicle_becomes_closer(self):
        selector = FirstVehicleSelector()
        first = vehicle(10, 5, 12.0)
        self.assertEqual(selector.select([first]).id, 10)

        first = vehicle(10, 6, 9.0, "stopping")
        later = vehicle(11, 1, 2.0)
        selected = selector.select([later, first])
        self.assertEqual(selected.id, 10)
        summary = general_summary([], [], [], [later, first], 1234, selected)
        self.assertEqual(summary["vehicle_status"], "stopping")
        self.assertEqual(summary["vehicle_distance"], 9.0)
        self.assertEqual(summary["vehicle_length"], 4.25)
        self.assertEqual(summary["vehicle_count"], 2)

    def test_chooses_oldest_track_then_reselects_when_it_disappears(self):
        selector = FirstVehicleSelector()
        first = vehicle(10, 7, 12.0)
        later = vehicle(11, 2, 2.0)
        self.assertEqual(selector.select([later, first]).id, 10)
        self.assertEqual(selector.select([later]).id, 11)
        self.assertIsNone(selector.select([]))
        self.assertIsNone(selector.selected_id)

    def test_direction_status_reaches_manager_summary_unchanged(self):
        for status in ("entering", "exiting"):
            selected = vehicle(10, 5, 9.0, status)
            summary = general_summary([], [], [], [selected], 1234, selected)
            self.assertEqual(summary["vehicle_status"], status)
            self.assertEqual(summary["vehicle_exist"], 1)

    def test_crane_sensor_header_and_invalid_timestamp(self):
        status = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1792821834, nanosec=326401792)),
            working_status="working", crane_moving=False, beam_length=2.0,
            beam_cluster_valid=True, work_complete=False, motion_dz=0.0)
        self.assertEqual(crane_summary(status)["sensor_timestamp_ms"], 1792821834326)
        for sec, ns in ((-1, 0), (1, -1), (1, 1000000000), (0, 0)):
            status.header.stamp.sec, status.header.stamp.nanosec = sec, ns
            self.assertEqual(crane_summary(status)["sensor_timestamp_ms"], 0)

    def test_crane_projection_preserves_readiness_without_recomputing_it(self):
        status = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=500000000)),
            working_status=0, crane_moving=False, beam_length=4.0,
            beam_cluster_valid=False, work_complete=False, motion_dz=0.0,
            prepare_ready=True, prepare_status="NOT_READY")
        summary = crane_summary(status)
        self.assertTrue(summary["prepare_ready"])
        self.assertEqual(summary["beam_length"], 0.0)
        self.assertEqual(summary["working_status"], "IDLE")
        self.assertEqual(summary["sensor_timestamp_ms"], 1500)
        self.assertNotIn("prepare_status", summary)
        for state in ("IDLE", "LIFTING", "SUSPENDED", "LOWERING", "COMPLETE_CONFIRM",
                      "SEARCH_WIRE", "TRACK_SHEAVE", "WAIT_CARGO", "READY",
                      "CONFIRM_CARGO", "LOCKED_TRACKING", "COMPLETED"):
            status.state = state
            status.length_locked = True
            status.beam_length = 3.8
            status.beam_length_locked = 4.0
            status.active_left_hook = True
            status.active_right_hook = False
            status.work_complete = state == "COMPLETE_CONFIRM"
            projected = crane_summary(status)
            self.assertEqual(projected["state"], "LOWERING_CONFIRMATION" if state == "COMPLETE_CONFIRM" else state)
            self.assertEqual(projected["beam_length"], 4.0)
            self.assertTrue(projected["active_left_hook"])
            self.assertFalse(projected["active_right_hook"])
            self.assertEqual(projected["work_complete"], status.work_complete)
        del status.prepare_ready
        self.assertFalse(crane_summary(status)["prepare_ready"])

    def test_lifting_and_measured_clearance_projection(self):
        status = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)),
            working_status=3, crane_moving=False, beam_length=3.0,
            beam_cluster_valid=True, work_complete=False, motion_dz=0.0,
            state="LOCKED_TRACKING", prepare_ready=True,
            beam_ground_clearance_valid=True, beam_ground_clearance_m=0.25,
            beam_bottom_z=1.25, ground_surface_z=1.0,
            initial_clearance_valid=True, initial_clearance_m=0.0, clearance_delta_m=0.25,
            beam_ground_clearance_sample_valid=[True, False],
            beam_ground_clearance_samples_m=[0.25, 0], beam_bottom_samples_z=[1.25, 0],
            ground_surface_samples_z=[1.0, 0])
        result = crane_summary(status)
        self.assertEqual(result["working_status"], "LIFTING")
        self.assertEqual(result["work_state"], "LIFTING")
        self.assertTrue(result["work_active"])
        self.assertTrue(result["prepare_ready"])
        self.assertEqual(result["state"], "LOCKED_TRACKING")
        self.assertEqual(result["beam_ground_clearance_samples_m"], [0.25, None])
        status.working_status = "lifting"
        status.beam_ground_clearance_valid = False
        self.assertIsNone(crane_summary(status)["beam_ground_clearance_m"])

    def test_crane_manual_output_preserves_real_measurement_validity(self):
        status = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)),
            working_status=1, crane_moving=False, beam_length=0.0,
            beam_cluster_valid=False, work_complete=False, motion_dz=0.0,
            state="LOWERING", auto_state="IDLE", prepare_ready=True,
            beam_detected=True, control_mode="MANUAL_OVERRIDE",
            manual_override="beam=true,working=1,fsm=LOWERING,ready=true")
        projected = crane_summary(status)
        self.assertTrue(projected["beam_detected"])
        self.assertFalse(projected["beam_cluster_valid"])
        self.assertEqual(projected["beam_length"], 0.0)
        self.assertEqual(projected["working_status"], "LOWERING")
        self.assertEqual(projected["state"], "LOWERING")
        self.assertEqual(projected["auto_state"], "IDLE")
        self.assertEqual(projected["control_mode"], "MANUAL_OVERRIDE")
        self.assertEqual(projected["manual_override"], status.manual_override)

    def test_same_age_uses_track_id_for_deterministic_choice(self):
        selector = FirstVehicleSelector()
        self.assertEqual(selector.select([vehicle(7, 1, 2.0), vehicle(3, 1, 8.0)]).id, 3)


if __name__ == "__main__":
    unittest.main()
