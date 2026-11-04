#!/usr/bin/env python3
"""
================================================================================
LiDAR Bringup 통합 런치 파일 (lidar_bringup.launch.py)
================================================================================
- TF 발행: YAML 설정 파일(config/lidar_tf.yaml)을 읽어 static_transform_publisher 실행
- 외부 LiDAR 센서 드라이버 실행 (지연 시작 지원)
- BEV 인지(Perception) 노드 실행
- RViz2 시각화 지원

* 기본 설정 변경은 상단의 [DEFAULT_CONFIG] 딕셔너리에서 바로 수정할 수 있습니다.
* 터미널 CLI 인자(예: ros2 launch ... use_rviz:=true)는 DEFAULT_CONFIG보다 우선 적용됩니다.
================================================================================
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ==============================================================================
# ⚙️ [1. 사용자 기본 설정 (DEFAULT CONFIGURATION)]
# ==============================================================================
# 터미널에서 별도 인자 없이 launch 실행 시 기본 적용되는 설정입니다.
# 자주 변경하는 옵션들을 여기서 편하게 수정하세요.
# ==============================================================================
DEFAULT_CONFIG = {
    "system_mode": "",  # Optional common profile; empty preserves legacy CLI defaults.
    # --------------------------------------------------------------------------
    # [1] TF / 센서 좌표계 설정 (YAML에서 좌표 및 frame_id를 직접 로드)
    # --------------------------------------------------------------------------
    "publish_tf": "true",                  # static TF 발행 여부 ('true' / 'false')
    # 기본 센서: "config/lidar_tf.yaml" | 크레인: "config/lidar_tf_crane.yaml"
    "tf_config": "config/lidar_tf.yaml",   # 사용할 YAML TF 설정 파일 경로

    # --------------------------------------------------------------------------
    # [2] 외부 LiDAR 센서 드라이버 설정
    # --------------------------------------------------------------------------
    "use_lidar_driver": "true",                  # LiDAR 드라이버 실행 여부 ('true' / 'false')
    "lidar_launch_package": "lslidar_driver",    # 드라이버 패키지명
    "lidar_launch_file": "lslidar_ch_launch.py", # 드라이버 launch 파일명
    "lidar_params_file": "",                     # 파라미터 파일 경로 (빈 문자열이면 패키지 기본값 사용)
    "lidar_params_argument_name": "params_file", # 드라이버 launch 인자명
    "lidar_namespace": "ch",                     # 센서 네임스페이스
    "pass_use_sim_time_to_lidar": "false",       # use_sim_time 인자 전달 여부
    "lidar_start_delay": "0.5",                  # TF 발행 후 드라이버 구동 대기시간 (초)

    # --------------------------------------------------------------------------
    # [3] BEV 인지 (Perception) 노드 설정
    # --------------------------------------------------------------------------
    "use_perception": "true",                    # 인지 노드 실행 여부 ('true' / 'false')
    "perception_launch_package": "cx128s2_bev_perception",
    "perception_launch_file": "cx128s2_bev_perception.launch.py",
    "perception_params_file": "",                # 인지 파라미터 파일 경로 (빈 문자열이면 기본값)
    "general_engine": "legacy",                  # Original General restored; V2 is experimental.
    "crane_engine": "v2",                        # Crane runtime; legacy is available for recovery.
    "perception_params_overlay_file": "",        # 선택적 실험 overlay (기본 동작 유지)
    "perception_input_topic": "",                # 입력 포인트클라우드 토픽 재정의 (기본: "")
    "perception_target_frame": "",               # 타겟 좌표계 재정의 (기본: "")
    "perception_start_delay": "1.0",             # 드라이버 구동 후 인지 노드 실행 대기시간 (초)
    "guardian_bridge_enabled": "true",
    "guardian_bridge_mode": "general",  # "crane" or "general"
    "guardian_bridge_manager_url": "http://127.0.0.1:8765/api/internal/bridge",
    "guardian_bridge_token": "",
    # Recorder remains idle until its fixed /start_recording service is called.
    "guardian_recorder_enabled": "false",
    "guardian_recorder_params_file": "",

    # --------------------------------------------------------------------------
    # [4] 시각화 (RViz2) 및 시뮬레이션 설정
    # --------------------------------------------------------------------------
    "use_rviz": "true",                         # RViz2 실행 여부 ('true' / 'false')
    "rviz_config": "rviz/lidar.rviz",            # RViz 설정 파일 경로
    "use_sim_time": "false",                     # 시뮬레이션 시간(/clock) 동기화 여부
}

# CLI 도움말 설명 (-s 옵션 시 노출)
ARGUMENT_DESCRIPTIONS = {
    "general_engine": "General perception engine: legacy (default) or v2 experimental",
    "crane_engine": "Crane perception engine: v2 (default) or legacy",
    # TF
    "publish_tf": "Publish static transform from YAML ('true'/'false')",
    "tf_config": "Path to YAML TF configuration file",
    # Driver
    "use_lidar_driver": "Launch external LiDAR driver ('true'/'false')",
    "lidar_launch_package": "Package name containing external LiDAR launch file",
    "lidar_launch_file": "External LiDAR launch file name",
    "lidar_params_file": "Optional path to external LiDAR parameters YAML file",
    "lidar_params_argument_name": "Launch argument name for params file in external launch",
    "lidar_namespace": "LiDAR ROS namespace",
    "pass_use_sim_time_to_lidar": "Pass use_sim_time argument to external LiDAR launch ('true'/'false')",
    "lidar_start_delay": "Delay (seconds) before launching external LiDAR driver",
    # Perception
    "use_perception": "Launch BEV perception node ('true'/'false')",
    "perception_launch_package": "Perception package name",
    "perception_launch_file": "Perception launch file name",
    "perception_params_file": "Optional override path to perception params file",
    "perception_params_overlay_file": "Optional perception YAML overlay (empty by default)",
    "perception_input_topic": "Optional input topic override for perception node",
    "perception_target_frame": "Optional target frame override for perception node",
    "perception_start_delay": "Delay (seconds) before launching perception node",
    "guardian_bridge_enabled": "Start the compact Guardian state bridge",
    "guardian_bridge_mode": "Bridge profile: crane or general",
    "guardian_bridge_manager_url": "Host manager internal bridge endpoint",
    "guardian_bridge_token": "Optional shared token for the bridge endpoint",
    "guardian_recorder_enabled": "Start the common on-demand PCD recorder",
    "guardian_recorder_params_file": "Optional recorder YAML override",
    # RViz & Sim Time
    "use_rviz": "Launch RViz2 for visualization ('true'/'false')",
    "rviz_config": "Path to RViz config file",
    "use_sim_time": "Use simulation time across all nodes ('true'/'false')",
}


# ==============================================================================
# 🛠️ [2. 유틸리티 및 경로 해석 함수]
# ==============================================================================
def str_to_bool(val) -> bool:
    """문자열 또는 불리언 값을 bool로 변환"""
    return str(val).strip().lower() in ("true", "1", "yes", "on")


def resolve_package_path(path_or_filename: str, package_name: str = "lidar_bringup") -> str:
    """
    절대경로면 그대로 반환하고,
    상대경로면 지정된 패키지의 share 디렉토리(또는 소스 폴더)를 기준으로 변환합니다.
    """
    if not path_or_filename:
        return ""
    if os.path.isabs(path_or_filename):
        return path_or_filename

    try:
        base_dir = get_package_share_directory(package_name)
    except PackageNotFoundError:
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    return os.path.join(base_dir, path_or_filename)


def validate_and_parse_yaml(tf_config_path: str) -> dict:
    """YAML TF 설정 파일을 파싱하고 필수 키 및 유효성을 검증합니다."""
    if not os.path.isfile(tf_config_path):
        raise FileNotFoundError(f"[lidar_bringup] YAML TF config file not found: '{tf_config_path}'")

    try:
        with open(tf_config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        raise ValueError(f"[lidar_bringup] Failed to parse YAML file '{tf_config_path}': {e}")

    if not isinstance(data, dict):
        raise ValueError(f"[lidar_bringup] YAML file '{tf_config_path}' does not contain a top-level dictionary.")

    for req_key in ["parent_frame", "child_frame", "translation", "rotation_rpy"]:
        if req_key not in data:
            raise ValueError(f"[lidar_bringup] YAML configuration missing required key: '{req_key}'")

    parent_frame = str(data["parent_frame"]).strip()
    child_frame = str(data["child_frame"]).strip()
    translation = data["translation"]
    rotation_rpy = data["rotation_rpy"]

    if not parent_frame:
        raise ValueError("[lidar_bringup] 'parent_frame' in YAML must be a non-empty string.")
    if not child_frame:
        raise ValueError("[lidar_bringup] 'child_frame' in YAML must be a non-empty string.")
    if parent_frame == child_frame:
        raise ValueError(f"[lidar_bringup] 'parent_frame' and 'child_frame' cannot be identical ('{parent_frame}').")

    for axis in ["x", "y", "z"]:
        if axis not in translation:
            raise ValueError(f"[lidar_bringup] 'translation' missing key: '{axis}'")
        val = translation[axis]
        if not isinstance(val, (int, float)) or isinstance(val, bool):
            raise TypeError(f"[lidar_bringup] 'translation.{axis}' must be numeric, got {type(val).__name__}")

    for axis in ["roll", "pitch", "yaw"]:
        if axis not in rotation_rpy:
            raise ValueError(f"[lidar_bringup] 'rotation_rpy' missing key: '{axis}'")
        val = rotation_rpy[axis]
        if not isinstance(val, (int, float)) or isinstance(val, bool):
            raise TypeError(f"[lidar_bringup] 'rotation_rpy.{axis}' must be numeric, got {type(val).__name__}")

    return {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "x": str(translation["x"]),
        "y": str(translation["y"]),
        "z": str(translation["z"]),
        "roll": str(rotation_rpy["roll"]),
        "pitch": str(rotation_rpy["pitch"]),
        "yaw": str(rotation_rpy["yaw"]),
    }


# ==============================================================================
# 🧩 [3. 하위 서브시스템 빌더 함수]
# ==============================================================================
def create_tf_action(context, use_sim_time_bool: bool):
    """YAML 설정 파일로부터 좌표/프레임을 읽어 static_transform_publisher 노드 생성"""
    if not str_to_bool(LaunchConfiguration("publish_tf").perform(context)):
        return []

    tf_config = resolve_package_path(LaunchConfiguration("tf_config").perform(context))
    if not tf_config:
        return []

    tf_data = validate_and_parse_yaml(tf_config)

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_lidar",
            arguments=[
                "--x", tf_data["x"],
                "--y", tf_data["y"],
                "--z", tf_data["z"],
                "--roll", tf_data["roll"],
                "--pitch", tf_data["pitch"],
                "--yaw", tf_data["yaw"],
                "--frame-id", tf_data["parent_frame"],
                "--child-frame-id", tf_data["child_frame"],
            ],
            parameters=[{"use_sim_time": use_sim_time_bool}],
            output="screen",
        )
    ]


def create_lidar_driver_action(context, use_sim_time_str: str):
    """외부 LiDAR 센서 드라이버 런치 액션 생성 (지연 실행 지원)"""
    use_lidar = str_to_bool(LaunchConfiguration("use_lidar_driver").perform(context))
    if not use_lidar:
        return []

    pkg = LaunchConfiguration("lidar_launch_package").perform(context).strip()
    launch_file = LaunchConfiguration("lidar_launch_file").perform(context).strip()
    if not pkg or not launch_file:
        return []

    try:
        pkg_share = get_package_share_directory(pkg)
    except PackageNotFoundError:
        raise RuntimeError(f"[lidar_bringup] External LiDAR package '{pkg}' not found in ROS workspace.")

    launch_path = os.path.join(pkg_share, "launch", launch_file)
    if not os.path.isfile(launch_path):
        raise FileNotFoundError(f"[lidar_bringup] External launch file not found: '{launch_path}'")

    args = {}
    ns = LaunchConfiguration("lidar_namespace").perform(context).strip()
    if ns:
        args["lidar_namespace"] = ns
        args["namespace"] = ns

    params_file = LaunchConfiguration("lidar_params_file").perform(context).strip()
    arg_name = LaunchConfiguration("lidar_params_argument_name").perform(context).strip()
    if params_file and arg_name:
        args[arg_name] = params_file

    if str_to_bool(LaunchConfiguration("pass_use_sim_time_to_lidar").perform(context)):
        args["use_sim_time"] = use_sim_time_str

    include_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=args.items(),
    )

    try:
        delay = float(LaunchConfiguration("lidar_start_delay").perform(context))
    except ValueError:
        delay = 0.5

    return [TimerAction(period=delay, actions=[include_action])] if delay > 0.0 else [include_action]


def create_perception_action(context, use_sim_time_str: str):
    """BEV 인지(Perception) 노드 런치 액션 생성 (지연 실행 지원)"""
    use_perc = str_to_bool(LaunchConfiguration("use_perception").perform(context))
    if not use_perc:
        return []

    pkg = LaunchConfiguration("perception_launch_package").perform(context).strip()
    launch_file = LaunchConfiguration("perception_launch_file").perform(context).strip()
    if not pkg or not launch_file:
        return []

    try:
        pkg_share = get_package_share_directory(pkg)
    except PackageNotFoundError:
        raise RuntimeError(f"[lidar_bringup] Perception package '{pkg}' not found in ROS workspace.")

    launch_path = os.path.join(pkg_share, "launch", launch_file)
    if not os.path.isfile(launch_path):
        raise FileNotFoundError(f"[lidar_bringup] Perception launch file not found: '{launch_path}'")

    args = {"use_sim_time": use_sim_time_str}
    if pkg == "cx128s2_bev_perception":
        crane = LaunchConfiguration("guardian_bridge_mode").perform(context).strip() == "crane"
        engine = LaunchConfiguration("crane_engine").perform(context).strip()
        if engine not in ("v2", "legacy"):
            raise ValueError("crane_engine must be v2 or legacy")
        general_engine = LaunchConfiguration("general_engine").perform(context).strip()
        if general_engine not in ("v2", "legacy"):
            raise ValueError("general_engine must be v2 or legacy")
        args["perception_engine"] = engine if crane else ("general_v2" if general_engine == "v2" else "legacy")
    if LaunchConfiguration("guardian_bridge_mode").perform(context).strip() == "crane":
        args["respawn"] = "true"
    params_file = LaunchConfiguration("perception_params_file").perform(context).strip()
    input_topic = LaunchConfiguration("perception_input_topic").perform(context).strip()
    target_frame = LaunchConfiguration("perception_target_frame").perform(context).strip()

    if params_file:
        args["params_file"] = params_file
    overlay = LaunchConfiguration("perception_params_overlay_file").perform(context).strip()
    if overlay:
        args["params_overlay_file"] = overlay
    if input_topic:
        args["input_topic"] = input_topic
    if target_frame:
        args["target_frame"] = target_frame

    include_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=args.items(),
    )

    try:
        delay = float(LaunchConfiguration("perception_start_delay").perform(context))
    except ValueError:
        delay = 1.0

    return [TimerAction(period=delay, actions=[include_action])] if delay > 0.0 else [include_action]


def create_guardian_bridge_action(context):
    """Start the Docker-local compact-state bridge for the selected profile."""
    if not str_to_bool(LaunchConfiguration("guardian_bridge_enabled").perform(context)):
        return []
    mode = LaunchConfiguration("guardian_bridge_mode").perform(context).strip()
    if mode not in ("crane", "general"):
        raise ValueError("guardian_bridge_mode must be 'crane' or 'general'")
    params = {
        "mode": mode,
        "manager_url": LaunchConfiguration("guardian_bridge_manager_url").perform(context),
        "bridge_token": LaunchConfiguration("guardian_bridge_token").perform(context),
    }
    input_topic = LaunchConfiguration("perception_input_topic").perform(context).strip()
    if mode == "crane" and input_topic:
        params["pointcloud_topic"] = input_topic
    return [TimerAction(period=1.5, actions=[Node(
        package="guardian_lidar_bridge", executable="guardian_lidar_bridge",
        name="guardian_lidar_bridge", parameters=[params], output="screen",
        respawn=(mode == "crane"), respawn_delay=2.0,
    )])]


def create_guardian_recorder_action(context):
    """Run the existing recorder so its fixed capture services are available."""
    if not str_to_bool(LaunchConfiguration("guardian_recorder_enabled").perform(context)):
        return []
    try:
        package_share = get_package_share_directory("cx128s2_bev_data_recorder")
    except PackageNotFoundError:
        raise RuntimeError("[lidar_bringup] cx128s2_bev_data_recorder package not found")
    params_file = LaunchConfiguration("guardian_recorder_params_file").perform(context).strip()
    if not params_file:
        params_file = os.path.join(package_share, "config", "cx128s2_bev_data_recorder.yaml")
    if not os.path.isfile(params_file):
        raise FileNotFoundError(f"[lidar_bringup] Recorder config not found: '{params_file}'")
    return [TimerAction(period=1.5, actions=[Node(
        package="cx128s2_bev_data_recorder", executable="cx128s2_bev_data_recorder_node",
        name="cx128s2_bev_data_recorder", parameters=[params_file], output="screen",
    )])]


def create_rviz_action(context, use_sim_time_bool: bool):
    """RViz2 시각화 노드 액션 생성"""
    if not str_to_bool(LaunchConfiguration("use_rviz").perform(context)):
        return []

    mode = LaunchConfiguration("system_mode").perform(context).strip()
    preset = "rviz/lidar_crane_work.rviz" if mode == "crane" else "rviz/lidar.rviz"
    rviz_config_raw = LaunchConfiguration("rviz_config").perform(context).strip() or preset
    rviz_config = resolve_package_path(rviz_config_raw)
    if not os.path.isfile(rviz_config):
        raise FileNotFoundError(f"[lidar_bringup] RViz config file not found: '{rviz_config_raw}'")

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time_bool}],
            output="screen",
        )
    ]


# ==============================================================================
# 🚀 [4. 메인 런치 설정 및 진입점]
# ==============================================================================
def launch_setup(context):
    """런타임 컨텍스트 평가 후 모든 노드 및 런치 액션 조립"""
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = str_to_bool(use_sim_time_str)

    mode = LaunchConfiguration("system_mode").perform(context).strip()
    if mode not in ("", "crane", "general"):
        raise ValueError("system_mode must be crane or general")
    if mode and LaunchConfiguration("guardian_bridge_mode").perform(context).strip() != mode:
        raise ValueError("system_mode and guardian_bridge_mode disagree")
    actions = []

    # 1. TF 액션 생성 (YAML 기반 static_transform_publisher)
    actions.extend(create_tf_action(context, use_sim_time_bool))

    # 2. LiDAR 센서 드라이버 액션 생성
    actions.extend(create_lidar_driver_action(context, use_sim_time_str))

    # 3. BEV 인지 노드 액션 생성
    actions.extend(create_perception_action(context, use_sim_time_str))

    # 4. Docker-local compact-state bridge.  It never exports PointCloud.
    actions.extend(create_guardian_bridge_action(context))
    actions.extend(create_guardian_recorder_action(context))

    # 5. RViz2 액션 생성
    actions.extend(create_rviz_action(context, use_sim_time_bool))

    # 실행 요약 로그 출력
    tf_config_val = LaunchConfiguration("tf_config").perform(context)
    publish_tf_val = LaunchConfiguration("publish_tf").perform(context)
    use_lidar = LaunchConfiguration("use_lidar_driver").perform(context)
    use_perc = LaunchConfiguration("use_perception").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    bridge_mode = LaunchConfiguration("guardian_bridge_mode").perform(context)

    tf_status = f"ON ({os.path.basename(tf_config_val)})" if str_to_bool(publish_tf_val) else "OFF"

    actions.insert(0, LogInfo(msg="===================================================="))
    actions.insert(1, LogInfo(msg=" [lidar_bringup] Pipeline Started"))
    actions.insert(2, LogInfo(msg=f"  - TF (YAML):    {tf_status}"))
    actions.insert(3, LogInfo(msg=f"  - LiDAR Driver: {'ON' if str_to_bool(use_lidar) else 'OFF'}"))
    actions.insert(4, LogInfo(msg=f"  - Perception:   {'ON' if str_to_bool(use_perc) else 'OFF'}"))
    actions.insert(5, LogInfo(msg=f"  - RViz2:        {'ON' if str_to_bool(use_rviz) else 'OFF'}"))
    actions.insert(6, LogInfo(msg=f"  - Sim Time:     {use_sim_time_str}"))
    actions.insert(7, LogInfo(msg=f"  - Guardian bridge: {bridge_mode}"))
    actions.insert(8, LogInfo(msg="===================================================="))

    return actions


def generate_launch_description():
    """런치 인자 선언 및 전체 런치 설명 생성"""
    launch_args = []

    # Resolve one explicitly selected mode into defaults. Individual path
    # arguments remain available for existing calibrated/custom installations.
    perception_share = get_package_share_directory("cx128s2_bev_perception")
    crane_defaults = {
        "tf_config": resolve_package_path("config/lidar_tf_crane.yaml"),
        "perception_params_file": os.path.join(perception_share, "config", "cx128s2_bev_perception_crane.yaml"),
        "perception_params_overlay_file": os.path.join(perception_share, "config", "crane_boundary_clustering_experimental.yaml"),
        "rviz_config": resolve_package_path("rviz/lidar_crane_work.rviz"),
        "guardian_bridge_mode": "crane",
    }
    for name, default_val in DEFAULT_CONFIG.items():
        if name in ("tf_config", "rviz_config"):
            default_val = resolve_package_path(default_val)
        if name in crane_defaults:
            default_val = PythonExpression([
                repr(crane_defaults[name]), " if '", LaunchConfiguration("system_mode"),
                "' == 'crane' else ", repr(default_val),
            ])
        launch_args.append(DeclareLaunchArgument(
            name, default_value=default_val,
            description=ARGUMENT_DESCRIPTIONS.get(name, "Common Crane/General mode" if name == "system_mode" else ""),
        ))

    # 이전 CLI 호환성을 위한 선택적 인자 (오류 방지용)
    launch_args.append(DeclareLaunchArgument("tf_source", default_value="yaml", description="Legacy argument (always uses yaml)"))

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
