"""ROS-independent compact payload projections, kept unit-testable."""
import math


VALID_VEHICLE_STATUSES = {"entering", "stopping", "stopped", "exiting"}


class FirstVehicleSelector:
    """Select the longest current vehicle; retain the import name for compatibility."""

    def __init__(self):
        self.selected_id = None

    def select(self, vehicles):
        candidates = [obj for obj in vehicles
                      if math.isfinite(float(obj.dimensions.x)) and float(obj.dimensions.x) > 0]
        # Length wins over distance, arrival order and prior selection. For equal
        # lengths keep the selected ID, then prefer older tracks and smaller IDs.
        selected = max(candidates, key=lambda obj: (
            float(obj.dimensions.x), int(obj.id) == self.selected_id,
            int(obj.age), -int(obj.id)), default=None)
        self.selected_id = int(selected.id) if selected is not None else None
        return selected


def general_summary(detected, tracked, motion, vehicles, sensor_timestamp_ms, selected_vehicle,
                    current=True):
    """Project the selected vehicle track to the manager's single-vehicle contract."""
    if not current:
        detected, tracked, motion, vehicles, selected_vehicle = [], [], [], [], None
    vehicle_exist = int(selected_vehicle is not None)
    vehicle_status = getattr(selected_vehicle, "vehicle_status", "stopping") if selected_vehicle else ""
    if selected_vehicle is not None and vehicle_status not in VALID_VEHICLE_STATUSES:
        vehicle_status = "stopping"
    vehicle_distance = float(selected_vehicle.distance) if selected_vehicle else 0.0
    vehicle_length = float(selected_vehicle.dimensions.x) if selected_vehicle else 0.0
    return {
        "object_detected": bool(detected),
        "object_count": len(detected),
        "tracked_object_count": len(tracked),
        "vehicle_count": len(vehicles),
        "moving_object_count": sum(
            1 for obj in motion if obj.motion_state != "STATIONARY"
        ),
        "vehicle_exist": vehicle_exist,
        "vehicle_status": vehicle_status,
        "vehicle_distance": vehicle_distance,
        "vehicle_length": vehicle_length,
        "sensor_timestamp_ms": int(sensor_timestamp_ms),
    }


def normalize_working_status(value):
    """Publish five FSM states; accept legacy API inputs during migration."""
    legacy = ("IDLE", "SUSPENDED", "LOWERING_CONFIRMATION", "LIFTING")
    if type(value) is int and 0 <= value < len(legacy):
        return legacy[value]
    if type(value) is str:
        value = value.upper()
        value = {"WORKING": "SUSPENDED", "DROP": "LOWERING_CONFIRMATION",
                 "COMPLETE_CONFIRM": "LOWERING_CONFIRMATION",
                 "LOWERING_CONFIRMED": "LOWERING_CONFIRMATION",
                 "LOWERING_CONFIRM": "LOWERING_CONFIRMATION"}.get(value, value)
        if value in ("IDLE", "LIFTING", "SUSPENDED", "LOWERING", "LOWERING_CONFIRMATION"):
            return value
    raise ValueError("working_status must be IDLE, LIFTING, SUSPENDED, LOWERING or LOWERING_CONFIRMATION")


def crane_summary(status):
    stamp = status.header.stamp
    sensor_ms = (stamp.sec * 1000 + stamp.nanosec // 1_000_000
                 if type(stamp.sec) is int and type(stamp.nanosec) is int and
                 0 <= stamp.sec <= 2147483647 and 0 <= stamp.nanosec < 1_000_000_000 else 0)
    # Detailed FSM state retains lowering/suspended distinctions lost by legacy work codes.
    state = str(getattr(status, "state", ""))
    if state in ("IDLE", "LIFTING", "SUSPENDED", "LOWERING", "LOWERING_CONFIRMATION",
                 "COMPLETE_CONFIRM", "LOWERING_CONFIRMED", "LOWERING_CONFIRM"):
        working = normalize_working_status(state)
        state = working
    else:
        working = normalize_working_status(status.working_status)
    valid = bool(getattr(status, "beam_ground_clearance_valid", False))
    initial_valid = bool(getattr(status, "initial_clearance_valid", False))
    flags = list(getattr(status, "beam_ground_clearance_sample_valid", []))
    samples = list(getattr(status, "beam_ground_clearance_samples_m", []))
    bottoms = list(getattr(status, "beam_bottom_samples_z", []))
    ground = list(getattr(status, "ground_surface_samples_z", []))
    if not len(flags) == len(samples) == len(bottoms) == len(ground):
        raise ValueError("inconsistent Beam-Ground measurement samples")
    return {
        "working_status": working,
        "work_state": working.upper(),
        "work_active": working in ("LIFTING", "SUSPENDED", "LOWERING"),
        "beam_ground_clearance_valid": valid,
        "beam_ground_clearance_m": float(status.beam_ground_clearance_m) if valid else None,
        "beam_bottom_z": float(status.beam_bottom_z) if valid else None,
        "ground_surface_z": float(status.ground_surface_z) if valid else None,
        "initial_clearance_valid": initial_valid,
        "initial_clearance_m": float(status.initial_clearance_m) if initial_valid else None,
        "clearance_delta_m": float(status.clearance_delta_m) if valid and initial_valid else None,
        "beam_ground_clearance_sample_valid": flags,
        "beam_ground_clearance_samples_m": [float(v) if ok else None for v, ok in zip(samples, flags)],
        "beam_bottom_samples_z": [float(v) if ok else None for v, ok in zip(bottoms, flags)],
        "ground_surface_samples_z": [float(v) if ok else None for v, ok in zip(ground, flags)],
        "beam_ground_clearance_reason": str(getattr(status, "beam_ground_clearance_reason", "UNKNOWN")),
        "crane_moving": int(status.crane_moving),
        "beam_length": float(status.beam_length_locked if getattr(status, "length_locked", False) else
                             status.beam_length if status.beam_cluster_valid else 0.0),
        "state": state or "IDLE",
        "active_left_hook": bool(getattr(status, "active_left_hook", False)),
        "active_right_hook": bool(getattr(status, "active_right_hook", False)),
        "sensor_timestamp_ms": int(sensor_ms),
        "beam_cluster_valid": bool(status.beam_cluster_valid),
        "work_complete": bool(status.work_complete),
        "motion_dz": float(status.motion_dz),
        "prepare_ready": bool(getattr(status, "prepare_ready", False)),
        "beam_detected": bool(getattr(status, "beam_detected", status.beam_cluster_valid)),
        "control_mode": str(getattr(status, "control_mode", "AUTO")),
        "auto_state": str(getattr(status, "auto_state", getattr(status, "state", "IDLE"))),
        "manual_override": str(getattr(status, "manual_override", "")),
    }
