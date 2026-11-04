# lidar_bringup

ROS 2 Jazzy 환경에서 다양한 LiDAR 센서를 유연하고 통합적으로 구동하기 위한 **범용 LiDAR Bringup 패키지**입니다.

---

## 주요 기능

1. **유연한 TF 발행 방식 지원**
   - **YAML 방식 (`tf_source:=yaml`)**: `tf2_ros static_transform_publisher` 사용
   - **URDF 방식 (`tf_source:=urdf`)**: `robot_state_publisher` + Xacro/URDF 해석 사용
2. **Launch 인자 덮어쓰기 (Overrides)**
   - `use_tf_launch_overrides:=true` 설정 시 CLI에서 전달한 `x`, `y`, `z`, `roll`, `pitch`, `yaw`, `parent_frame`, `child_frame` 인자가 파일 설정값보다 우선 적용됩니다.
3. **외부 LiDAR Driver 구동 연동**
   - TF가 발행된 후 지정된 시간(`lidar_start_delay`)만큼 대기하여 드라이버 초기화 순서를 보장하고 외부 LiDAR launch 파일을 동적으로 포섭(`IncludeLaunchDescription`)합니다.
4. **검증 및 안전 장치**
   - YAML/URDF 파일 존재 여부, 필수 키 검증, 데이터 타입 검증, 부모/자식 frame 중복 검증 수행.
5. **중복 TF 방지 지침**
   - URDF 방식과 YAML 방식은 상호 배타적으로 동작합니다.

---

## 1. 패키지 구조

```text
lidar_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── lidar_bringup.launch.py
├── config/
│   └── lidar_tf.yaml
├── urdf/
│   └── lidar_description.urdf.xacro
├── rviz/
│   └── lidar.rviz
└── README.md
```

---

## 2. 빌드 및 환경 설정

```bash
cd /root/ros2_ws # 또는 사용자의 워크스페이스 경로
colcon build --symlink-install --packages-select lidar_bringup
source install/setup.bash
```

---

## 3. Launch 인자 안내

| Launch Argument | 기본값 | 설명 |
| :--- | :--- | :--- |
| `publish_tf` | `true` | YAML 설정 기반 static TF 발행 여부 (`true` / `false`) |
| `tf_config` | `<pkg_share>/config/lidar_tf.yaml` | YAML TF 설정 파일 경로 (`config/lidar_tf.yaml` 또는 `config/lidar_tf_crane.yaml`) |
| `use_lidar_driver` | `true` | 외부 LiDAR 센서 드라이버 실행 여부 (`true` / `false`) |
| `lidar_launch_package` | `lslidar_driver` | 구동할 외부 LiDAR 드라이버 패키지명 |
| `lidar_launch_file` | `lslidar_ch_launch.py` | 외부 LiDAR launch 파일명 |
| `lidar_params_file` | `""` | 외부 LiDAR 드라이버 파라미터 YAML 파일 경로 |
| `lidar_params_argument_name`| `params_file` | 외부 launch에서 파라미터 파일에 사용하는 launch 인자명 |
| `lidar_namespace` | `ch` | LiDAR ROS 2 네임스페이스 |
| `pass_use_sim_time_to_lidar`| `false` | 외부 LiDAR launch로 `use_sim_time` 인자 전달 여부 |
| `lidar_start_delay` | `0.5` | static TF 발행 후 LiDAR launch 실행까지 대기 시간(초) |
| `use_perception` | `true` | BEV Perception 노드(`cx128s2_bev_perception`) 구동 여부 |
| `perception_launch_package` | `cx128s2_bev_perception` | Perception 패키지명 |
| `perception_launch_file` | `cx128s2_bev_perception.launch.py` | Perception launch 파일명 |
| `perception_params_file` | `""` | Perception 파라미터 YAML 파일 경로 (빈값 시 기본 설정 사용) |
| `perception_input_topic` | `""` | Perception 입력 토픽 덮어쓰기 옵션 |
| `perception_target_frame` | `""` | Perception 타겟 프레임 덮어쓰기 옵션 |
| `perception_start_delay` | `1.0` | static TF 및 드라이버 후 Perception 실행까지 대기 시간(초) |
| `use_rviz` | `false` | RViz2 구동 여부 (`true` / `false`) |
| `rviz_config` | `<pkg_share>/rviz/lidar.rviz` | RViz 설정 파일 경로 |
| `use_sim_time` | `false` | 시뮬레이션 타임 사용 여부 |

---

## 4. 주요 실행 예시

### 4.1 YAML TF + 실제 LiDAR 실행 (CX128S2 예시)

```bash
ros2 launch lidar_bringup lidar_bringup.launch.py \
  tf_source:=yaml \
  tf_config:=/root/ros2_ws/src/lidar_bringup/config/lidar_tf.yaml \
  lidar_launch_package:=lslidar_driver \
  lidar_launch_file:=lslidar_ch_launch.py \
  lidar_params_file:=/root/ros2_ws/src/lslidar_driver/config/lslidar_ch.yaml \
  lidar_namespace:=ch
```

### 4.2 URDF TF + 실제 LiDAR 실행

```bash
ros2 launch lidar_bringup lidar_bringup.launch.py \
  tf_source:=urdf \
  description_file:=/root/ros2_ws/src/lidar_bringup/urdf/lidar_description.urdf.xacro \
  lidar_launch_package:=lslidar_driver \
  lidar_launch_file:=lslidar_ch_launch.py
```

### 4.3 TF 값 직접 덮어쓰기 (Launch CLI Overrides)

```bash
ros2 launch lidar_bringup lidar_bringup.launch.py \
  tf_source:=yaml \
  use_tf_launch_overrides:=true \
  parent_frame:=world \
  child_frame:=laser_link \
  x:=0.0 \
  y:=3.0 \
  z:=7.014126 \
  roll:=-2.287096604 \
  pitch:=0.950310530 \
  yaw:=-0.523598776
```

### 4.4 RViz 포함 실행

```bash
ros2 launch lidar_bringup lidar_bringup.launch.py \
  tf_source:=yaml \
  use_rviz:=true
```

---

## 5. 핵심 동작 매커니즘 및 검증 가이드

### 5.1 TF 발행 방향 (Transform Direction)
발행되는 정적 변환(Static Transform)은 **`world` -> `laser_link`** 방향입니다.
- **Parent Frame**: `world` (기준 고정 좌표계)
- **Child Frame**: `laser_link` (LiDAR 센서 물리 위치 좌표계)
- 위치(XYZ) 단위는 **미터(m)**이며, 회전(RPY) 단위는 **라디안(rad)**입니다.

### 5.2 URDF 방식 vs YAML 방식 실행 차이
- **YAML 방식 (`tf_source:=yaml`)**:
  - `config/lidar_tf.yaml`을 파싱하여 필수 검증(존재 여부, 수치 타입, 부모/자식 동일 여부 등)을 마친 후 `tf2_ros`의 `static_transform_publisher` 노드를 개별 구동합니다.
  - 빠르게 정적 좌표 변환 노드만 실행하고자 할 때 유용합니다.
- **URDF 방식 (`tf_source:=urdf`)**:
  - `urdf/lidar_description.urdf.xacro` 또는 일반 `.urdf` 파일을 파싱(xacro 변환 수행)하여 `robot_description` 파라미터를 생성한 뒤 `robot_state_publisher` 노드를 실행합니다.
  - 차후 전체 로봇 모델(URDF 트리가 복잡한 로봇)과 결합하거나 URDF 기반 시각화를 선호할 때 적합합니다.

### 5.3 외부 LiDAR Launch 포섭 및 파라미터 전달
- `IncludeLaunchDescription` 및 `PythonLaunchDescriptionSource`를 활용해 외부 패키지(`lidar_launch_package`)의 launch 파일(`lidar_launch_file`)을 실행합니다.
- `lidar_params_file` 및 `lidar_params_argument_name` 인자를 통해 외부 launch 파일이 요구하는 이름(기본값: `params_file`)으로 설정 파일 경로를 전달합니다.
- `TimerAction(period=lidar_start_delay)`을 통해 TF 노드가 완전히 뜬 후 LiDAR 드라이버 노드가 구동되므로, TF 수신 오류나 시간 동기화 문제를 예방합니다.

### 5.4 중복 TF 및 이중 좌표 변환 방지 정책

> [!WARNING]
> LiDAR 드라이버 내부 좌표 변환 기능과 외부 정적 TF 발행을 동시에 적용하면 점군(PointCloud) 좌표계가 **이중으로 변환**되어 위치 오류가 발생할 수 있습니다.

1. **launch 내 배타적 실행**: `tf_source`에 따라 URDF(`robot_state_publisher`) 방식 또는 YAML(`static_transform_publisher`) 방식 중 하나만 실행되며, 동시 발행되지 않습니다.
2. **드라이버 내부 변환 비활성화 권장**: 외부 static TF (`world` -> `laser_link`)를 사용하는 경우, `lslidar_driver` 설정 파라미터 YAML(`lslidar_ch.yaml`)에서 아래 값을 `false`로 설정하는 것을 권장합니다.

```yaml
is_pretreatment: false
is_MatrixTransformation: false
```

### 5.5 TF 발행 검증 방법

아래 명령어로 `world`에서 `laser_link`로의 정적 변환이 올바르게 발행되는지 확인할 수 있습니다.

```bash
ros2 run tf2_ros tf2_echo world laser_link
```

정상 실행 시 다음과 같은 변환 정보가 지속적으로 출력됩니다:

```text
At time 1721750000.000000000
- Translation: [0.000, 3.000, 7.014]
- Rotation: in Quaternion [-0.887, 0.285, 0.082, 0.353]
- Rotation: in RPY (radian) [-2.287, 0.950, -0.524]
```

### 5.6 PointCloud topic `frame_id` 확인 방법

발행되는 PointCloud2 메시지(`/ch/lslidar_point_cloud`)의 `frame_id`가 `laser_link`인지 확인하려면 다음 명령을 실행합니다:

```bash
ros2 topic echo --once /ch/lslidar_point_cloud /header
```

출력 결과예시:

```yaml
stamp:
  sec: 1721750000
  nanosec: 123456789
frame_id: laser_link
```
`frame_id`가 `laser_link`로 표시되면 static TF와 완벽하게 연동됩니다.
