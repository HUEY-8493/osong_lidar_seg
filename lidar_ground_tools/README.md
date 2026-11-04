# lidar_ground_tools

ROS 2 Humble Python tools for fixed LiDAR ground-plane calibration and calibrated bag generation.

이 패키지는 PCD 기반으로 고정식 LiDAR의 ground-plane calibration을 1회성으로 수행하고, 산출된 보정값을 PointCloud2 rosbag에 적용하기 위한 도구이다.

## Tools

### `calibrate_lidar_ground_once`

Reads a directory of PCD files, fits a ground plane with numpy-only RANSAC, estimates `z/roll/pitch`, keeps `x/y/yaw` as manual inputs, and writes:

- `lidar_ground_calibration.yaml`
- `lidar_ground_calibration_debug.png`
- `lidar_scan_calibration_applied_view.png`
- `lidar_first_person_view.png`

Example:

```bash
cd ~/HS_folder/deto_develope_series
ros2 run lidar_ground_tools calibrate_lidar_ground_once \
  --pcd-dir ./pcd \
  --manual-x 0.0 \
  --manual-y 3.0 \
  --manual-yaw-deg -25.0 \
  --sample-count 15
```

### `apply_lidar_calibration_to_bag`

Reads a PointCloud2 rosbag2 bag, applies the calibration YAML transform to the `x/y/z` fields, preserves all other point fields, and writes a new bag.

Example:

```bash
cd ~/HS_folder/deto_develope_series
ros2 run lidar_ground_tools apply_lidar_calibration_to_bag \
  --input-bag ./pcd_bag \
  --output-bag ./pcd_bag_calibrated \
  --calibration-yaml ./lidar_ground_calibration.yaml \
  --topic /pcd_pointcloud \
  --overwrite
```

By default the corrected PointCloud2 `header.frame_id` is set to the YAML `parent_frame`, usually `map`.

## Development Checklist

- [x] PCD 파일에서 `x/y/z` pointcloud 데이터 읽기 기능 구현
- [x] 유효 z 범위 및 최대 point 수 기준으로 point filtering 처리
- [x] RANSAC 기반 지면 plane 추정 기능 구현
- [x] 여러 PCD frame에서 추정된 plane을 robust하게 평균화하는 로직 구현
- [x] 지면 plane normal을 이용해 LiDAR roll / pitch 계산
- [x] 지면 기준 높이를 이용해 LiDAR z(height) 계산
- [x] LiDAR x / y / yaw는 수동 입력값으로 적용
- [x] 추정된 LiDAR z / roll / pitch와 수동 입력 x / y / yaw를 YAML로 저장
- [x] 보정 결과 검증용 debug plot 생성
- [x] 보정 적용 후 pointcloud view plot 생성
- [x] LiDAR 기준 first-person view plot 생성
- [x] 보정 YAML을 적용해 PointCloud2 rosbag을 다시 생성하는 코드 구현

## Progress Notes

- PCD 기반으로 LiDAR 설치 자세를 1회성으로 추정하는 보정 스크립트를 개발했다.
- 현재 방식은 LiDAR의 z 값을 미리 알고 있는 것이 아니라, world 기준 지면 높이 `ground_z`를 알고 있다는 전제에서 동작한다.
- 기본적으로 `ground_z = 0.0`을 기준면으로 두고, PCD pointcloud 내 지면 plane을 RANSAC으로 추정한 뒤 LiDAR의 높이 `z`, `roll`, `pitch`를 계산한다.
- 지면 plane은 LiDAR의 `roll`, `pitch`, `height`는 제약할 수 있지만, `x`, `y`, `yaw`는 관측할 수 없다.
- 따라서 `x`, `y`, `yaw`는 수동 입력값을 사용하도록 구성했다.
- 좌표계 기준은 `x forward`, `y left`, `z up`으로 정리했다.
- 여러 PCD frame을 사용해 각 frame별 ground plane을 추정하고, median 기반으로 최종 plane을 계산하도록 했다.
- 추정 결과는 `lidar_ground_calibration.yaml`로 저장되며, `roll/pitch/yaw` degree 값과 validation 결과를 함께 기록한다.
- 보정 품질 확인을 위해 ground z error histogram, plane residual histogram, 보정 적용 후 scan view, LiDAR first-person view 이미지를 생성하도록 했다.
- 추가로 `lidar_ground_calibration.yaml`의 transform을 PointCloud2 bag에 적용해 보정된 `pcd_bag_calibrated`를 생성하는 스크립트를 구현했다.
- 현재 개발 범위는 ground plane 기반 자동 보정값 산출과 보정 bag 생성까지이며, 실제 TF 반영 및 현장 기준 검증은 추가 확인이 필요하다.

## Calibration Principle

이 보정은 지면을 하나의 평면으로 보고, LiDAR 좌표계에서 관측된 지면 평면이 world 좌표계의 `z = ground_z` 평면에 오도록 LiDAR의 자세와 높이를 역산하는 방식이다.

LiDAR pointcloud에서 지면 점들은 다음 평면 방정식을 만족한다고 둔다.

```text
a*x + b*y + c*z + d = 0
```

여기서 `n = [a, b, c]`는 LiDAR 좌표계에서 본 지면 평면의 normal vector이다. RANSAC을 사용해 outlier를 제외하고 지면에 해당하는 dominant plane을 찾는다. 이후 여러 PCD frame에서 얻은 normal과 `d` 값을 median 기반으로 평균화해 최종 ground plane을 만든다.

LiDAR 좌표계는 다음과 같이 둔다.

```text
x: forward
y: left
z: up
```

ROS의 roll-pitch-yaw 회전 순서를 다음과 같이 사용한다.

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

이때 world z축 방향은 LiDAR 좌표계에서 회전행렬의 세 번째 행으로 표현된다. 이 값은 다음과 같다.

```text
[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
```

지면 평면의 normal vector `n = [nx, ny, nz]`는 LiDAR 좌표계에서 본 world z축 방향으로 해석할 수 있다. 따라서 다음 관계를 둔다.

```text
[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)] = [nx, ny, nz]
```

이 관계에서 roll과 pitch를 계산하면 다음과 같다.

```text
pitch = asin(-nx)
roll  = atan2(ny, nz)
```

Yaw는 이 식에 나타나지 않는다. 즉 지면 평면만으로는 yaw를 알 수 없다. 그래서 yaw는 사용자가 수동으로 입력한다.

높이 z는 평면 방정식의 `d` 값으로부터 계산한다. 변환 후 어떤 LiDAR 점 `p_lidar`의 world z는 다음과 같다.

```text
world_z = n^T * p_lidar + lidar_z
```

지면 위의 점은 평면 방정식 때문에 다음을 만족한다.

```text
n^T * p_lidar + d = 0
```

즉,

```text
n^T * p_lidar = -d
```

지면 점이 world에서 `ground_z`에 와야 하므로,

```text
ground_z = -d + lidar_z
```

따라서 LiDAR 높이는 다음과 같다.

```text
lidar_z = ground_z + d
```

단, 평면 normal은 부호가 두 가지로 표현될 수 있다.

```text
n*x + d = 0
```

과

```text
(-n)*x + (-d) = 0
```

은 같은 평면이다. 그래서 코드에서는 두 부호 후보 중 `lidar_z`가 양수가 되고, 변환 후 ground inlier들의 world z가 `ground_z`에 가장 잘 맞는 방향을 선택한다.

이후 검증 단계에서는 RANSAC으로 선택된 지면 inlier들을 실제로 world 좌표계로 변환한다. 변환된 지면 점들의 z 값이 `ground_z` 근처에 모이면 보정이 잘 된 것이다. 이를 위해 다음 지표를 계산한다.

```text
mean_z_error
median_z_error
max_abs_z_error
std_z_error
```

현재 결과에서 `median_z_error`가 수 mm 수준이면, 추정된 `roll`, `pitch`, `z`가 지면 평면을 world의 `z=0` 기준면에 잘 정렬했다는 의미이다.

정리하면 이 방법으로 알 수 있는 값과 알 수 없는 값은 다음과 같다.

```text
추정 가능:
- LiDAR height z
- LiDAR roll
- LiDAR pitch

추정 불가능:
- LiDAR x
- LiDAR y
- LiDAR yaw
```

그 이유는 하나의 무한 평면은 기울기와 평면까지의 거리는 제공하지만, 평면 위에서의 이동량 `x/y`와 평면 법선 주변 회전 `yaw`는 구속하지 않기 때문이다.

짧게 말하면 이 방법은 plane-to-plane alignment의 특수한 경우이다. LiDAR에서 관측된 ground plane을 world의 기준 ground plane `z = ground_z`에 정렬하는 rigid transform 중, ground plane만으로 관측 가능한 자유도인 `roll`, `pitch`, `z`만 추정하고, 관측 불가능한 `x`, `y`, `yaw`는 manual prior로 고정한다.

## Limits and Checks

- Ground plane만으로는 `x`, `y`, `yaw`를 자동 추정할 수 없다.
- `ground_z` 기준면이 잘못되어 있으면 LiDAR height도 그만큼 bias를 가진다.
- RANSAC이 실제 바닥이 아니라 벽, 사면, 구조물의 큰 평면을 잡으면 보정 결과가 틀어진다.
- Debug plot과 applied scan view에서 ground inlier가 `z = ground_z` 근처에 모이는지 반드시 확인해야 한다.
- 실제 TF에 반영하기 전에는 현장 기준 좌표계, LiDAR driver 좌표축 convention, map frame 정의를 추가 검증해야 한다.
