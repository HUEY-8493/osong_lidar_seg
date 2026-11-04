# PCD sequence player

The default directory is the local recording at `/home/sjjin/pcd_cloud` (315 frames). Override `pcd_directory` for another recording. For `timing_mode:=recorded`, place `frames.csv` in the PCD directory or its parent.

```bash
cd /home/sjjin/construct_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pcd_replay
source install/setup.bash
ros2 run pcd_replay pcd_sequence_player --ros-args -p fps:=10.0 -p loop:=true
```

Parameters: `pcd_directory`, `topic` (default `/ch/lslidar_point_cloud`), `frame_id` (default `laser_link`), `fps` (default `10.0`), `loop` (default `true`), `start_frame` (default `0`), `end_frame` (default `-1`), `timing_mode` (`fixed` or `recorded`), `paused` (startup state). Frame indices are zero based and the end is inclusive. `paused` is only a startup setting; live pause/step controls are not implemented. `recorded` uses the CSV timestamps for intervals, while message stamps always use the current ROS clock. At a loop boundary, it waits `1/fps` before starting again.

In another terminal, source ROS and the workspace, then run `rviz2`. Set Fixed Frame to `laser_link`, add a PointCloud2 display, and set Topic to `/ch/lslidar_point_cloud`. Select the Intensity color transformer for this recording. To inspect the topic, use `ros2 topic info /ch/lslidar_point_cloud`, `ros2 topic echo /ch/lslidar_point_cloud --once --field header.frame_id`, and `ros2 topic hz /ch/lslidar_point_cloud`.

The player keeps at most two PCD frames queued for reading and never loads the entire sequence. It supports binary and ASCII PCD, preserves supported scalar and array fields, and rejects `binary_compressed` with a clear error. The provided recording is binary. The `frames.csv` from this recording has malformed final quoted fields, so recorded timing reads its numeric prefix and raw PCD filename per physical line.
