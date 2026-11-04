# cx128s2_bev_perception

CX128S2 3D 라이다 기반 고성능 BEV(Bird's Eye View) 인지 패키지입니다.  
본 패키지는 건설 현장 타워크레인 상부 센싱 환경을 위한 **Crane 모드**와, 도로변/게이트 지주 고정 센싱 환경을 위한 **General 모드**의 두 가지 독립적인 인지 파이프라인을 제공합니다.

---

## 1. 패키지 디렉토리 구조

```text
cx128s2_bev_perception/
├── README.md                                      # [본 문서] 프로젝트 전체 안내 및 AI 개발 가이드
├── docs/                                          # 상세 기술 문서
│   ├── crane.md                                   # Crane 모드 기술 문서 (Top-Down Chain, Z-Gaussian, 안정화)
│   └── general.md                                 # General 모드 기술 문서 (배경 차분, 차량 추적, 재획득)
├── include/cx128s2_bev_perception/                # C++ 헤더 파일
│   ├── crane_pipeline.hpp                         # Crane 슬로프/Z-가우시안/Top-Down 파이프라인 선언
│   ├── crane_beam_detector.hpp                    # [Legacy/호환용] 기존 빔 검출기 선언
│   ├── crane_hook_detector.hpp                    # [Legacy/호환용] 기존 훅 검출기 선언
│   ├── linear_kalman_filter.hpp                   # 선형 등속 칼만 필터
│   ├── raw_vehicle_box_refinement.hpp             # 3D 점군 기반 차량 OBB 보정
│   ├── vehicle_surface_tracking.hpp               # 차량 생애주기, 표면 매칭, 재획득 알고리즘
│   └── visibility_model.hpp                       # 라이다 빔 채널 광선 가시성 모델
├── src/                                           # C++ 구현 파일
│   ├── cx128s2_bev_perception_node.cpp            # 메인 ROS 2 인지 노드 (콜백, 모드 분기, 토픽 발행)
│   ├── crane_pipeline.cpp                         # Crane 파이프라인 핵심 구현체
│   ├── crane_beam_detector.cpp                    # [Legacy] 기존 빔 검출기 구현
│   ├── crane_hook_detector.cpp                    # [Legacy] 기존 훅 검출기 구현
│   └── visibility_model.cpp                       # 가시성 모델 구현
├── config/                                        # 설정 파일
│   ├── cx128s2_bev_perception_crane.yaml          # Crane 기본 설정
│   ├── crane_boundary_clustering_experimental.yaml# Crane 실험/최신 슬로프 및 체인 오버레이 설정
│   ├── cx128s2_bev_perception.yaml                # General 기본 설정
│   ├── cx128s2_roi_policy.yaml                    # General 다중 영역 ROI 정책
│   └── cx128s2_static_background.yaml             # General 정적 배경 맵
├── launch/                                        # ROS 2 실행 파일
│   └── cx128s2_bev_perception.launch.py           # 인지 노드 단독 런치
└── test/                                          # C++ 단위 테스트
    ├── test_crane_pipeline.cpp                    # Crane 슬로프, Top-Down 체인, 안정화 단위 테스트
    ├── test_tracking_policy.cpp                   # General 차량 추적 단위 테스트
    └── test_vehicle_surface_tracking.cpp          # 재획득 및 생애주기 단위 테스트
```

---

## 2. Crane 모드와 General 모드 비교

두 모드는 단일 노드 실행 파일(`cx128s2_bev_perception_node`)을 공유하지만, 파라미터 `perception_mode`에 의해 완전히 다른 데이터 처리 흐름을 수행합니다.

| 구분 | Crane 모드 (`crane_beam_work`) | General 모드 (`default`) |
|---|---|---|
| **설치 환경** | 타워크레인/호이스트 상부 (하향 뷰) | 도로변 지주(Pole) 또는 게이트 (사선 뷰) |
| **주요 목적** | Hook Root 검출, Top-Down 와이어 연결선 추적, Largest Connected Cluster 추출 | 정적 배경 차분, 진입/진출 차량 검출, 칼만 추적, 가림 차량 재획득 |
| **ROI 정책** | `crane_beam.beam_roi` 및 Hook 영역 사전 필터링 | `cx128s2_roi_policy.yaml` 기반 다중 영역 (WATCH/IGNORE/HARD_IGNORE) |
| **지면 처리** | 지면 기준 상대 고도 계산 (연결 시 지면 아래까지 탐색 지속) | 정적 배경 맵 제거 또는 격자 최소 높이 기반 지면 제거 |
| **클러스터링** | 1D Z-Gaussian 레이어 분리 후 레이어별 BEV CC 클러스터링 | 단일 전경 점군 대상 BEV CC (4-way, Z-Guard, Gap-Bridging) |
| **핵심 추적** | Hook Root 칼만 추적, 지향성 하향 트리 탐색, 체인 M-of-N 확정 | Constant-Velocity 선형 칼만 추적, ACTIVE/LOST/DORMANT 생애주기 |
| **특수 제약** | **절대 기울기($\le 35.0^\circ$) 및 회전각($\le 40.0^\circ$) 제한** | **차량 규격 필터링, Lateral Anchor, 표면 점군 재획득 스코어링** |
| **작업/화물 판정**| **명시적 비활성화 (체결 여부 미판정, `working_status=0` 고정)** | N/A (차량 객체 검출 및 차선 통과 방향 추정) |
| **상세 문서** | 📖 [docs/crane.md](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/docs/crane.md) | 📖 [docs/general.md](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/docs/general.md) |

---

## 3. 공통 알고리즘 및 코드 공유 관계

| 기능 모듈 | Crane | General | 코드 공유 여부 및 구현 위치 |
|---|:---:|:---:|---|
| **TF2 좌표 변환** | O | O | **동일 코드 공유:** `node.cpp: pointCloudCallback()` |
| **입력 범위 Crop (`passValidCrop`)** | O | O | **동일 코드 공유:** `node.cpp: passValidCrop()` |
| **2D BEV Connected Component** | O | O | **엔진 인스턴스 공유 (`clusterer_`):** Crane은 Z-레이어별로 반복 호출 |
| **선형 칼만 필터 (`LinearKalmanFilter`)**| O | O | **클래스 공유:** `linear_kalman_filter.hpp` (Crane은 Hook Root/말단, General은 차량에 적용) |
| **2D Convex Hull 및 Bounding Box** | O | O | **동일 로직 공유:** 중심점, 외접 사각형 계산 |
| **Z-Gaussian 레이어링** | O | X | **Crane 전용:** `crane_pipeline.cpp: processZGaussianGrouping()` |
| **Top-Down Hook Chain & 제한** | O | X | **Crane 전용:** `crane_pipeline.cpp: processHookAndBeamDetection()` |
| **Largest Connected Cluster** | O | X | **Crane 전용:** `crane_pipeline.cpp: processHookAndBeamDetection()` |
| **시간적 안정화 (Voxel 누적 & 구조 Hold)** | O | X | **Crane 전용:** `crane_pipeline.cpp: updateAccumulatedPoints(), stabilizeTopDownStructure()` |
| **정적 배경 학습 및 통계 점수 전경 분리** | X | O | **General 전용:** `node.cpp: accumulateBackground(), classifyForeground()` |
| **차량 생애주기 (LOST/DORMANT) 및 재획득** | X | O | **General 전용:** `vehicle_surface_tracking.hpp: scoreReacquisition()` |
| **가시성 모델 (Ray Occlusion Shadow)** | X | O | **General 전용:** `visibility_model.cpp: VisibilityModel` |

---

## 4. 실행 진입점 및 Bringup 방법

통합 Bringup은 상위 패키지인 `lidar_bringup`을 통해 실행하는 것을 권장합니다.

### 4.1 Crane 모드 실행 (Production)
```bash
# 1. 시뮬레이션 / Rosbag 재생 환경 (외부 드라이버 미실행, RViz2 자동 실행)
ros2 launch lidar_bringup lidar_crane_bringup.launch.py use_lidar_driver:=false

# 2. 실제 라이다 하드웨어 구동 환경
ros2 launch lidar_bringup lidar_crane_bringup.launch.py use_lidar_driver:=true
```
- **적용 TF:** `lidar_bringup/config/lidar_tf_crane.yaml`
- **적용 설정:** `cx128s2_bev_perception_crane.yaml` + `crane_boundary_clustering_experimental.yaml` (Overlay)
- **적용 RViz:** `lidar_bringup/rviz/lidar_crane.rviz`

### 4.2 General 모드 실행 (Production)
```bash
# 1. 시뮬레이션 / Rosbag 재생 환경
ros2 launch lidar_bringup lidar_bringup.launch.py use_lidar_driver:=false

# 2. 실제 라이다 하드웨어 구동 환경
ros2 launch lidar_bringup lidar_bringup.launch.py use_lidar_driver:=true
```
- **적용 TF:** `lidar_bringup/config/lidar_tf.yaml`
- **적용 설정:** `cx128s2_bev_perception.yaml`
- **적용 RViz:** `lidar_bringup/rviz/lidar.rviz`

### 4.3 인지 노드 단독 실행
```bash
# Crane 단독
ros2 launch cx128s2_bev_perception cx128s2_bev_perception.launch.py \
  params_file:=/home/sjjin/construct_ws/src/cx128s2_bev_perception/config/cx128s2_bev_perception_crane.yaml \
  params_overlay_file:=/home/sjjin/construct_ws/src/cx128s2_bev_perception/config/crane_boundary_clustering_experimental.yaml

# General 단독
ros2 launch cx128s2_bev_perception cx128s2_bev_perception.launch.py \
  params_file:=/home/sjjin/construct_ws/src/cx128s2_bev_perception/config/cx128s2_bev_perception.yaml
```

---

## 5. 설정 파일 우선순위 (Configuration Precedence)

파라미터는 다음 순서로 덮어쓰기(Overlay) 적용됩니다:

1. **기본 설정 (`params_file`):** `cx128s2_bev_perception.yaml` 또는 `cx128s2_bev_perception_crane.yaml`
2. **실험 오버레이 (`params_overlay_file`):** `crane_boundary_clustering_experimental.yaml` (기본값 재정의)
3. **런타임 CLI 인자 (`runtime_overrides`):** `input_topic`, `target_frame`, `use_sim_time` (최우선 적용)

---

## 6. 주요 ROS Topic 요약

### 6.1 Crane 모드
- **입력:** `/ch/lslidar_point_cloud` (`sensor_msgs/msg/PointCloud2`)
- **시각화 마커:** `/crane_hook/markers` (`visualization_msgs/msg/MarkerArray`)
  - `hook_root`: 상부 Hook Root (구체)
  - `chain_direct_line`: 연속 관측된 하향 연결선 (실선)
  - `chain_gap_line`: 결손 추정 구간 또는 Hold 구간 연결선 (호박색 점선)
  - `hook_terminal`: 체인 최하단 노드 (구체)
  - `largest_connected_box`: 최대 연결 클러스터 3D OBB (산호색 큐브)
  - `hook_info_text`: Hook 상태, 각도 통계, 클러스터 규격 텍스트
- **외부 호환 상태:** `/crane_beam/status` (`cx128s2_bev_perception/msg/CraneBeamStatus`)
- **작업 상태:** `/crane_beam/working_status` (`std_msgs/msg/Int32`, **항상 0**)
- **진단 스트링:** `/crane_slope/boundary_diagnostics` (`std_msgs/msg/String`, JSON 포맷)
- **디버그 점군:** `/crane_slope/z_gaussian_layers`, `/crane_slope/boundary_clusters`

### 6.2 General 모드
- **입력:** `/ch/lslidar_point_cloud` (`sensor_msgs/msg/PointCloud2`)
- **전경 점군:** `/debug/foreground_points` (`sensor_msgs/msg/PointCloud2`)
- **차량 검출 배열:** `/detected_objects` (`cx128s2_bev_perception/msg/TrackedObjectArray`)
- **활성 추적 배열:** `/tracked_objects`, `/road_vehicle_tracks`
- **장기 기억 식별자:** `/vehicle_identity_memory` (DORMANT 포함)
- **시각화 마커:** `/debug/object_markers`, `/debug/visibility_markers`
- **진단 테이블:** `/debug/track_table`, `/debug/candidate_table`

---

## 7. AI 개발자를 위한 중요 개발 규칙 (Strict Guidelines)

향후 AI 에이전트가 본 코드베이스를 수정하거나 기능을 추가할 때 반드시 준수해야 하는 원칙입니다.

1. **Crane 모드에서 화물 체결 여부 및 Work Status 판정을 임의로 활성화하지 마십시오:**
   - 현재 Crane 파이프라인의 명시적 목표는 **Top-Down 연결 구조와 Largest Connected Cluster를 관측 및 시각화**하는 것입니다.
   - 바닥에 닿았다거나 빔 형태가 관측되었다고 해서 `working_status = 1`로 변경하거나 `LOADED` 상태로 확정하는 로직을 추가하지 마십시오.
2. **지면 접촉을 탐색 종료 조건으로 사용하지 마십시오:**
   - 와이어와 훅이 지면 또는 적재물에 닿아 있더라도 기하학적 연결이 유지되는 한 최하단까지 연속 탐색해야 합니다.
3. **기존 검증된 함수와 구조체를 재사용하십시오:**
   - Hook 체인은 `CraneHookChain`, 노드는 `CraneHookChainNode`, 안정화는 `stabilizeTopDownStructure()`를 반드시 활용하십시오.
   - 독자적인 중복 추적기나 헬퍼 클래스를 새로 생성하지 마십시오.
4. **수평 빔 접촉 평가 시 유효 접촉 좌표 클램핑을 유지하십시오:**
   - 긴 물체($L > 5\text{m}$)의 경우 중심점(Centroid) 기준 계산을 수행하면 인위적인 측면 기울기가 발생하여 기각됩니다. Bounding Box 범위 내로 클램핑된 $\text{eff\_x}, \text{eff\_y}$를 사용하십시오.
5. **실시간 10Hz 연산 제약을 준수하십시오:**
   - 전체 파이프라인 C++ 처리 시간은 반드시 $40\text{ms}$ 미만을 유지해야 합니다.
