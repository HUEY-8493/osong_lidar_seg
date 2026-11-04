std_msgs/Header header

# FSM state: "IDLE", "LIFTING", "SUSPENDED", "LOWERING", "LOWERING_CONFIRM"
string state

# Public work status: IDLE / LIFTING / SUSPENDED / LOWERING / LOWERING_CONFIRM.
string working_status

# Motion reference mode used: "absolute" or "relative_background"
string motion_reference_mode

# Measurements
float32 z_beam
float32 z_background
float32 dz_beam
float32 dz_background
float32 relative_dz
float32 motion_dz

float32 beam_mad
float32 background_mad

uint32 beam_roi_point_count
uint32 background_roi_point_count
uint32 beam_cluster_point_count

# Beam length
float32 beam_length
float32 beam_length_locked
bool length_locked
float32 beam_major_axis_yaw

# Validities
bool beam_z_valid
bool background_z_valid
bool beam_cluster_valid

# Status flags
bool crane_moving
bool placed_candidate
bool work_complete

# Measurement source: "INITIAL_ROI" or "TRACKED_CLUSTER"
string beam_measurement_source

# Detailed Z bounds
float32 beam_z_min
float32 beam_z_max

# Centroid in target_frame
float32 beam_centroid_x
float32 beam_centroid_y

# Tracking diagnostics
uint32 tracking_candidate_count
float32 selected_tracking_score

# Local Support Surface & Placement
bool support_valid
float32 support_z
uint32 support_point_count
float32 placement_gap_z

# Contact & Tilted Touchdown
bool contact_candidate
float32 supported_segment_ratio

# Cumulative motion
float32 motion_cumulative_dz

# Crane Hook measurements used by the existing work FSM
bool left_hook_detected
float32 left_hook_x
float32 left_hook_z
float32 left_ground_z
# Measured body-band bottom Z minus valid local ground Z [m]; zero if invalid.
float32 left_hook_ground_distance

bool right_hook_detected
float32 right_hook_x
float32 right_hook_z
float32 right_ground_z
# Same bottom-ground definition as left_hook_ground_distance.
float32 right_hook_ground_distance

float32 mean_hook_ground_distance
# Preparation latch inside IDLE; never a separate FSM state.
bool prepare_ready
# Debug label derived from prepare_ready: READY / NOT_READY.
string prepare_status

# Hook CV tracking. detected/x/z above remain current measurement only.
# filtered_z and vz are usable only when tracked is true (MEASURED/PREDICTED).
bool left_hook_tracked
string left_hook_tracking_state
uint32 left_hook_lost_frames
float32 left_hook_filtered_z
float32 left_hook_vz

bool right_hook_tracked
string right_hook_tracking_state
uint32 right_hook_lost_frames
float32 right_hook_filtered_z
float32 right_hook_vz

# Existing cycle identity and measured/predicted/lost tracking diagnostics.
bool active_left_hook
bool active_right_hook
bool left_prepare_ready
bool right_prepare_ready
string beam_tracking_state
uint32 beam_lost_frames

# Final output control. Measurement validity/geometry above always remain automatic.
string control_mode
bool beam_detected
string auto_state
string auto_working_status
bool auto_beam_detected
bool auto_prepare_ready
bool auto_work_complete
# Per-field overrides; AUTO means that field still follows perception.
string manual_override

# Continuous Hook position; never body measurement or work-FSM evidence.
string left_hook_position_tracking_state
bool left_hook_position_valid
bool left_hook_position_updated
geometry_msgs/Point left_hook_tracking_position
bool left_hook_fallback_point_valid
geometry_msgs/Point left_hook_fallback_position
string right_hook_position_tracking_state
bool right_hook_position_valid
bool right_hook_position_updated
geometry_msgs/Point right_hook_tracking_position
bool right_hook_fallback_point_valid
geometry_msgs/Point right_hook_fallback_position

# Public work projection; existing state retains the detailed Cargo/Beam FSM phase.
string work_state
bool work_active
# Current direct surfaces, world Z-up. Unknown scalar values are ignored unless valid.
bool beam_ground_clearance_valid
float32 beam_ground_clearance_m
float32 beam_bottom_z
float32 ground_surface_z
bool initial_clearance_valid
float32 initial_clearance_m
float32 clearance_delta_m
float32[] beam_ground_clearance_samples_m
float32[] beam_bottom_samples_z
float32[] ground_surface_samples_z
bool[] beam_ground_clearance_sample_valid
string beam_ground_clearance_reason
