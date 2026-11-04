#pragma once
#include "cx128s2_bev_perception/v2/core.hpp"
#include <cx128s2_bev_perception/msg/crane_beam_status.hpp>
namespace crane_v2 {
inline cx128s2_bev_perception::msg::CraneBeamStatus status(const Output&o, const std_msgs::msg::Header&header)
{
  cx128s2_bev_perception::msg::CraneBeamStatus m;
  m.header = header;
  m.header.frame_id = "world";
  m.state = o.state;
  m.working_status = o.state;
  m.auto_state = o.state;
  m.auto_working_status = o.state;
  m.motion_reference_mode = "absolute";
  m.control_mode = "AUTO";
  m.manual_override = "AUTO";
  m.beam_detected = o.cargo.direct;
  m.auto_beam_detected = o.cargo.direct;
  m.beam_cluster_valid = o.cargo.direct;
  m.beam_z_valid = o.cargo.direct;
  m.beam_measurement_source = "TRACKED_CLUSTER";
  m.beam_tracking_state = o.cargo.direct?"MEASURED":o.cargo.held?"PREDICTED":"LOST";
  m.beam_centroid_x = o.cargo.center.x;
  m.beam_centroid_y = o.cargo.center.y;
  m.z_beam = o.cargo.center.z;
  m.beam_length = o.cargo.length_locked ? o.cargo.model_length : o.cargo.length;
  m.beam_length_locked = o.cargo.model_length;
  m.length_locked = o.cargo.length_locked;
  m.beam_major_axis_yaw = o.cargo.yaw;
  m.beam_cluster_point_count = o.cargo.raw_indices.size();
  m.beam_roi_point_count = o.cargo.raw_indices.size();
  m.beam_z_min = o.cargo.direct?o.cargo.raw_z_min:0;
  m.beam_z_max = o.cargo.direct?o.cargo.raw_z_max:0;
  m.motion_dz = o.motion_dz;
  m.relative_dz = o.motion_dz;
  m.dz_beam = o.motion_dz;
  // Independent motion activity, never inferred from the Cargo FSM.
  m.crane_moving = o.crane_moving;
  m.work_complete = o.work_complete;
  m.auto_work_complete = o.work_complete;
  m.work_active = o.work_active;
  m.work_state = o.work_active?"WORKING":"IDLE";
  m.beam_ground_clearance_valid = o.clearance_valid;
  m.beam_ground_clearance_m = o.clearance;
  m.beam_bottom_z = o.cargo.bottom_z;
  m.ground_surface_z = o.ground.z;
  m.support_valid = o.clearance_valid;
  m.support_z = o.ground.z;
  m.support_point_count = o.ground.raw_indices.size();
  m.background_z_valid = o.ground.valid;
  m.z_background = o.ground.z;
  m.background_roi_point_count = o.ground.raw_indices.size();
  m.background_mad = o.ground.mad;
  m.placement_gap_z = o.clearance;
  m.beam_ground_clearance_reason = o.clearance_valid?"V2_ESTIMATED_BODY_BOTTOM_AND_LOCAL_PLANE":"V2_BOTTOM_OR_GROUND_UNKNOWN";
  m.beam_ground_clearance_samples_m = {float(o.clearance)};
  m.beam_bottom_samples_z = {float(o.cargo.bottom_z)};
  m.ground_surface_samples_z = {float(o.ground.z)};
  m.beam_ground_clearance_sample_valid = {o.clearance_valid};
  m.contact_candidate = o.reason == "GROUND_CONTACT";
  m.tracking_candidate_count = o.candidates;
  const auto&l = o.hooks[0];
  const auto&r = o.hooks[1];
  m.left_hook_detected = l.direct;
  m.left_hook_x = l.virtual_hook.x;
  m.left_hook_z = l.virtual_hook.z;
  m.left_hook_tracked = l.direct||l.held;
  m.left_hook_tracking_state = l.direct?"MEASURED":l.held?"PREDICTED":"LOST";
  m.left_hook_filtered_z = l.virtual_hook.z;
  m.right_hook_detected = r.direct;
  m.right_hook_x = r.virtual_hook.x;
  m.right_hook_z = r.virtual_hook.z;
  m.right_hook_tracked = r.direct||r.held;
  m.right_hook_tracking_state = r.direct?"MEASURED":r.held?"PREDICTED":"LOST";
  m.right_hook_filtered_z = r.virtual_hook.z;
  m.active_left_hook = o.cargo.supporting_side == 0;
  m.active_right_hook = o.cargo.supporting_side == 1;
  m.left_hook_position_tracking_state = m.left_hook_tracking_state;
  m.right_hook_position_tracking_state = m.right_hook_tracking_state;
  m.left_hook_position_valid = m.left_hook_tracked;
  m.right_hook_position_valid = m.right_hook_tracked;
  m.left_hook_position_updated = l.direct;
  m.right_hook_position_updated = r.direct;
  m.left_hook_tracking_position.x = l.virtual_hook.x;
  m.left_hook_tracking_position.y = l.virtual_hook.y;
  m.left_hook_tracking_position.z = l.virtual_hook.z;
  m.right_hook_tracking_position.x = r.virtual_hook.x;
  m.right_hook_tracking_position.y = r.virtual_hook.y;
  m.right_hook_tracking_position.z = r.virtual_hook.z;
  if (o.clearance_valid)
  {
    if (m.active_left_hook)
    {
      m.left_ground_z = o.ground.z;
      m.left_hook_ground_distance = o.clearance;
    }
    if (m.active_right_hook)
    {
      m.right_ground_z = o.ground.z;
      m.right_hook_ground_distance = o.clearance;
    }
    m.mean_hook_ground_distance = o.clearance;
  }
  m.prepare_status = "NOT_READY";
  return m;
}
}
