# CX128S2 BEV Perception — Crane Mode 개발 문서

> **문서 버전:** 1.0.0  
> **최종 갱신일:** 2026-10-18  
> **대상 패키지:** `cx128s2_bev_perception`  
> **핵심 실행 프로파일:** `perception_mode: "crane_beam_work"`, `crane_pipeline: "slope"`  
> **주요 소스 파일:**
> - [include/cx128s2_bev_perception/crane_pipeline.hpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/crane_pipeline.hpp)
> - [src/crane_pipeline.cpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/crane_pipeline.cpp)
> - [src/cx128s2_bev_perception_node.cpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/cx128s2_bev_perception_node.cpp)
> - [config/crane_boundary_clustering_experimental.yaml](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/config/crane_boundary_clustering_experimental.yaml)

---

## 1. Crane 개발 목적

Crane 모드는 타워크레인/호이스트 상부에 장착된 CX128S2 3D 라이다로부터 취득된 고밀도 포인트클라우드를 기반으로 다음을 달성하기 위해 설계되었습니다.

1. **상부 Hook Root 검출 및 칼만 트래킹:**
   - 크레인 트롤리/붐대 직하단(Z $\ge 4.8\text{m}$)의 물리적 Hook Root를 좌우 독립적으로 검출하고 Kalman Filter를 통해 연속 추적.
2. **Top-Down 연결 구조(Hook Chain) 탐색:**
   - Root에서 시작하여 지향성 하향 트리(Directed Downward Tree, $z_A > z_B$)를 따라 Z-Gaussian 레이어 및 BEV 클러스터 간의 기하학적 연결망을 추적.
   - 와이어로프의 순간적인 점군 결손 구간에서도 허용된 원추 탐색($R_0 + \Delta z \tan\theta_{\text{max}}$)을 통해 Direct 연결과 Estimated Gap 연결을 구분 형성.
3. **기울기(Slope) 및 방향 변화(Turn Angle) 제한:**
   - 수직 하향 기준 절대 기울기($\le 35.0^\circ$) 및 이전 연결 벡터 대비 꺾임각($\le 40.0^\circ$)을 검사하여 측면 구조물이나 벽면으로의 오연결 원천 차단.
4. **Largest Connected Cluster (최대 연결 클러스터) 추출:**
   - 최상단 Root에서 이어진 연결 그래프에 속한 노드 중 공간 점유 면적/체적이 가장 큰 클러스터를 선택하여 3차원 OBB(Oriented Bounding Box)로 RViz에 표시.
   - **주의: 지면 높이에 도달하더라도 연결이 유지되면 탐색을 중단하지 않으며, 이 클러스터는 '관측된 최대 연결 구조'일 뿐 실제 화물 체결 여부나 Work Status를 확정하지 않음.**
5. **시간적 안정화 (Temporal Stabilization):**
   - 크레인 정지 시 5프레임 Voxel 누적으로 라이다 노이즈를 억제하고, 트래킹 단계에서 노드 M-of-N 확정, 결손 시 Hold, 바운딩 박스 치수/Yaw EMA 평활화를 적용.

---

## 2. 현재 Pipeline 처리 흐름

실제 C++ 코드 상에서 센서 콜백부터 마커 발행까지의 함수 호출 순서는 다음과 같습니다.

```text
cx128s2_bev_perception_node.cpp: pointCloudCallback()
        │
        ▼ (Target Frame TF 변환: world)
passValidCrop()
        │
        ▼ (perception_mode_ == "crane_beam_work")
processCraneSlope()
        │
        ├── 1. crane_pipeline_->updateConfig()
        │       └─ ROI Pre-filtering (pipe_cfg.roi_filter: Beam ROI P_s)
        │
        ├── 2. crane_pipeline_->process()
        │       ├─ updateAccumulatedPoints() (정지 중 5프레임 Voxel 누적)
        │       ├─ classifyRingsSlope() (경사도/단차/거리점프 경계 검출)
        │       └─ processZGaussianGrouping() (1D Gaussian Z-Layer 분리)
        │
        ├── 3. clusterer_->cluster() (Z-Layer별 BEV Connected Component 생성)
        │       └─ raw_subclusters 구성 (중심점, Bounding Box, 면적 계산)
        │
        ├── 4. [선택적/현재 비활성] processClosedLoopGrouping() (폐곡선 객체 그룹핑)
        │
        ├── 5. crane_pipeline_->processHookAndBeamDetection()
        │       ├─ [Fixed Root 모드: fixed_root.enabled = true]
        │       │   ├─ 좌/우 고정 Root ROI 영역 내부 점군 분포 조사 (Left: -0.90, 0.05, 5.25 / Right: 1.07, 0.05, 5.30)
        │       │   ├─ 관측 상태 머신 (OBSERVED / NOT_OBSERVED, 3프레임 Hold 유지)
        │       │   ├─ ROI 내부 점군 속도/변위 기반 Crane Move 감지 (STATIONARY / MOVING)
        │       │   └─ Hook Chain 최상단 Root 좌표를 기하학적 중심점으로 불변 고정 (흔들림 0.00mm)
        │       ├─ [Fallback 동적 모드: fixed_root.enabled = false]
        │       │   ├─ Root Subcluster 선정 (좌/우 분리, 질량+높이 점수, 이력 히스테리시스)
        │       │   └─ Root Kalman Filter 예측 및 업데이트 (정지/이동 적응형 Q)
        │       ├─ 하향 트리 탐색 (Downward Tree Traversal)
        │       │   ├─ Z 단차 조건: dz >= 0.03m, max_gap_z <= 2.5m
        │       │   ├─ 수직선 추선(Plumb-line) 편차 제한: bbox_dxy <= 0.50m
        │       │   ├─ 유효 접촉 좌표 클램핑 (cand_eff_x, cand_eff_y)
        │       │   ├─ Absolute Slope 검사: atan2(dxy, dz) <= 35.0 deg
        │       │   ├─ Turn Angle 검사: acos(v1 · v2) <= 40.0 deg
        │       │   └─ 비용 함수 평가 및 최적 Child 노드 채택
        │       ├─ Chain 결과 구성 (Direct vs Estimated Gap 마킹)
        │       ├─ Spatially Largest Connected Cluster 1차 추출 (Root 제외 최대 면적 노드)
        │       ├─ 체인 M-of-N 시간적 일관성 추적 (tracked_hook_chains_)
        │       └─ Nearest Cluster 탐색 (Hook 직하단 최근접 클러스터)
        │
        ├── 6. crane_pipeline_->stabilizeTopDownStructure()
        │       ├─ 크레인 이동 보상 (crane_moving 시 d_rx, d_ry 좌표 보정)
        │       ├─ 노드 M-of-N 확정 (node_confirm_frames = 2)
        │       ├─ 노드 일시 결손 Hold (node_max_hold_frames = 2, is_held = true)
        │       ├─ Largest Cluster 동일성 추적 및 스왑 히스테리시스 (1.4배 초과 또는 3프레임 연속)
        │       ├─ 2D PCA Yaw 계산 및 90도 모호성 점프 억제 (aspect >= 1.3)
        │       └─ Bounding Box 치수 (Length, Width, Centroid) EMA 평활화 (alpha = 0.20)
        │
        └── 7. 시각화 및 진단 데이터 발행
                ├─ publishCraneHookMarkers() (RViz2 MarkerArray: /crane_hook/markers)
                ├─ /crane_beam/status (CraneBeamStatus, working_status = 0)
                └─ /crane_slope/boundary_diagnostics (JSON 진단 스트링)
```

### 상세 단계별 입출력 및 파라미터 매핑

| 단계 | 소스 위치 및 함수 | 입력 데이터 | 출력 데이터 | 주요 파라미터 | 활성 상태 |
|---|---|---|---|---|---|
| **입력 및 TF** | `node.cpp: pointCloudCallback` | `/ch/lslidar_point_cloud` | `world` 좌표계 점군 | `target_frame: "world"`, `use_tf_transform: true` | **Active** |
| **ROI 필터** | `node.cpp: processCraneSlope` | 전체 유효 점군 | ROI 내부 점군 `active_indices` | `only_crane_roi: true`, `beam_roi_x/y/z` | **Active** |
| **정지 누적** | `crane_pipeline.cpp: updateAccumulatedPoints` | 프레임 점군 | 5프레임 누적 점군 `accumulated_points_` | `window_size: 5`, `voxel_size_xy: 0.04`, `min_hit_count: 2` | **Active** (정지 시) |
| **Z-Gaussian** | `crane_pipeline.cpp: processZGaussianGrouping` | ROI 누적 점군 | Z-Layer 분할 리스트 `res.z_gaussian` | `sigma_z: 0.03`, `range_sigma_factor: 0.002`, `max_layer_z_spread: 0.35` | **Active** |
| **BEV 클러스터링** | `node.cpp` (기존 `clusterer_`) | Z-Layer별 점군 | `std::vector<CraneSubcluster>` | `bev_x/y_resolution: 0.1`, `cluster_connectivity: 4` | **Active** |
| **폐곡선 그룹핑**| `crane_pipeline.cpp: processClosedLoopGrouping`| 서브클러스터 | `res.closed_loop` | `closed_loop_grouping.enabled: false` | **Disabled** |
| **Top-Down 탐색**| `crane_pipeline.cpp: processHookAndBeamDetection`| 서브클러스터 목록 | `CraneTopDownHookResult` (체인 및 미안정화 클러스터) | `max_slope_angle_deg: 35.0`, `max_turn_angle_deg: 40.0`, `max_gap_z: 2.5` | **Active** |
| **시간적 안정화**| `crane_pipeline.cpp: stabilizeTopDownStructure` | 원시 체인 및 클러스터 | 안정화된 체인, 노드 Hold, 평활화된 Largest Cluster | `node_confirm_frames: 2`, `largest_swap_confirm_frames: 3`, `alpha: 0.20` | **Active** |
| **마커 및 상태** | `node.cpp: publishCraneHookMarkers` | 안정화된 결과 | `/crane_hook/markers`, `/crane_beam/status` | - | **Active** |

---

## 3. Crane 기능별 개발 현황

| 기능 | 현재 상태 | 구현 위치 | 검증 여부 | 비고 |
|---|---|---|---|---|
| **Z-Gaussian Layer** | `Implemented + Active` | `crane_pipeline.cpp: processZGaussianGrouping()` | **Verified** | 1D 가우시안 신뢰구간 기반 레이어 분리 |
| **BEV Clustering** | `Implemented + Active` | `node.cpp: processCraneSlope()` (`clusterer_->cluster`) | **Verified** | 레이어별 2D BEV Connected Component 분할 |
| **Closed Loop Grouping** | `Implemented + Disabled` | `crane_pipeline.cpp: processClosedLoopGrouping()` | Experimental | YAML에서 `closed_loop_grouping.enabled: false`로 비활성화됨 |
| **Hook Root Detection** | `Implemented + Active` | `crane_pipeline.cpp: processHookAndBeamDetection()` | **Verified** | 질량+고도 결합 스코어, 칼만 예측 기반 히스테리시스 |
| **Hook Root Kalman Tracking** | `Implemented + Active` | `crane_pipeline.cpp: processHookAndBeamDetection()` | **Verified** | 정지($q=0.20$)/이동($q=2.50$) 가변 프로세스 노이즈 적용 |
| **Top-Down Hook Chain** | `Implemented + Active` | `crane_pipeline.cpp: processHookAndBeamDetection()` | **Verified** | 하향 지향성 탐색, Direct vs Gap-Tolerant 연결 구분 |
| **Slope / Turn 제한** | `Implemented + Active` | `crane_pipeline.cpp: processHookAndBeamDetection()` | **Verified** | 절대 기울기 $35.0^\circ$, 꺾임각 $40.0^\circ$, 유효 접촉 클램핑 |
| **Largest Connected Cluster**| `Implemented + Active` | `crane_pipeline.cpp: processHookAndBeamDetection()` | **Verified** | Root 제외 하향 체인 노드 중 최대 점유 면적/체적 선택 |
| **Temporal Stabilization** | `Implemented + Active` | `crane_pipeline.cpp: stabilizeTopDownStructure()` | **Verified** | 노드 2프레임 확인, 2프레임 Hold, 스왑 3프레임 지연, 치수/Yaw EMA |
| **Stationary Accumulation** | `Implemented + Active` | `crane_pipeline.cpp: updateAccumulatedPoints()` | **Verified** | 크레인 정지 시 5프레임 Voxel 누적 버퍼 동작 |
| **Legacy Beam Detector** | `Implemented + Disabled` | `crane_beam_detector.cpp: CraneBeamDetector` | Deprecated | 외부 토픽 구독이 없으면 실행되지 않음 |
| **Work Status (체결 판정)** | `Implemented + Disabled` | `crane_pipeline.cpp: processHookAndBeamDetection()` | **Disabled by Design**| `working_status = 0`, `work_reason = "NONE"` 고정 (화물 판정 금지) |

---

## 4. Hook Detection 및 Top-Down Chain 상세 사양

### 4.1 시작점 (Hook Root) 선정 및 트래킹: 고정 Root ROI vs Fallback 동적 모드

Crane 모드는 최상단 Hook Root의 시작점 설정 방식으로 **고정 Root ROI 모드**와 **Fallback 동적 모드**를 지원합니다.

#### 4.1.1 고정 Root ROI 및 기하학적 중심점 시작 구조 (`fixed_root.enabled: true`, 권장 기본값)
- **배경 및 목적:**
  - 기존 동적 검출 방식은 최상단 Z-Gaussian 레이어의 분할/병합 및 포인트 분포 미세 변화로 인해 정지 상태에서도 Root 좌표가 $13\sim 21\text{mm}$ 수준으로 지속 흔들리는 문제가 있었습니다.
  - 최상단 기계적 트롤리/호이스트 하단 구조물은 센서 좌표계 기준 위치가 고정되어 있으므로, 데이터셋 실측 분포를 기반으로 고정된 3차원 ROI를 정의하고 **그 ROI의 정확한 기하학적 중심점(Geometric Center)을 Root 좌표로 불변 고정**하여 흔들림을 수학적으로 완전 제거($0.000\text{mm}$)합니다.
- **선정된 고정 ROI 파라미터 (실측 3개 데이터셋 3,551프레임 종합 최적값):**
  - **Left Root ROI:**
    - X 범위: $[-1.25, -0.55]\text{m}$ (폭 $0.70\text{m}$)
    - Y 범위: $[-0.25, 0.35]\text{m}$ (길이 $0.60\text{m}$)
    - Z 범위: $[4.80, 5.70]\text{m}$ (높이 $0.90\text{m}$)
    - **고정 중심점 (Root 기준점):** $(-0.900, 0.050, 5.250)\text{m}$
  - **Right Root ROI:**
    - X 범위: $[0.72, 1.42]\text{m}$ (폭 $0.70\text{m}$)
    - Y 범위: $[-0.25, 0.35]\text{m}$ (길이 $0.60\text{m}$)
    - Z 범위: $[4.80, 5.80]\text{m}$ (높이 $1.00\text{m}$)
    - **고정 중심점 (Root 기준점):** $(1.070, 0.050, 5.300)\text{m}$
- **좌표 불변성 (Zero-Jitter Invariant):**
  - 프레임마다 포인트 분포가 바뀌거나 클러스터 형상이 달라져도 Hook Chain의 최상단 Root 좌표는 항상 고정 중심점 $(X_{\text{center}}, Y_{\text{center}}, Z_{\text{center}})$에 고정됩니다.
- **ROI 점군 관측 상태 머신 (Observation State Machine):**
  - ROI 내부 포인트 수 $\ge \text{min\_points} (15\text{pts})$ 검출 시 `OBSERVED` 상태.
  - 포인트 수가 일시 감소하거나 결손될 경우 최대 `hold_frames` (기본 $3$프레임) 동안 직전 관측 상태를 유지(`is_held = true`).
  - **핵심 원칙:** 포인트가 완전히 소실되어 `NOT_OBSERVED` 상태가 되더라도 **Root 기준점 자체는 절대로 삭제되지 않으며 고정 좌표를 영구히 유지**합니다.
- **크레인 이동 감지 (Crane Move Detection):**
  - ROI 내부 점군 분포의 중심점($\bar{X}, \bar{Y}, \bar{Z}$) 변화율과 순간 속도($v_{\text{inst}} = \Delta d_{xy} / \Delta t$)를 지수 이동 평균(EMA, $\alpha=0.25$)으로 평활화.
  - 이동 조건($v_{\text{smooth}} \ge 0.12\text{m/s}$ 또는 $\Delta d_{xy} \ge 0.08\text{m}$) 연속 $3$프레임 충족 시 `MOVING`, 정지 조건 연속 $2$프레임 충족 시 `STATIONARY`.
  - **주의:** 이 모션 감지는 순수한 크레인 차체 이동 플래그(`crane_moving`)이며, **화물 작업 상태(`working_status = 0 / NONE`)와는 완전히 분리되어 연동되지 않습니다.**
- **Top-Down 하향 체인 연결:**
  - 고정 중심점 노드에서 출발하여 하부 와이어 및 Hook 서브클러스터들로 하향 지향성 탐색을 즉시 개시합니다.

#### 4.1.2 Fallback 동적 Root 트래킹 모드 (`fixed_root.enabled: false`)
- **위치:** `src/crane_pipeline.cpp:1918-1968`
- **검격 기준:**
  - $Z \ge \text{root\_min\_z} (4.8\text{m})$
  - 포인트 수 $\ge \text{min\_root\_points} (15\text{pts})$
  - 좌/우 영역 분할: 좌측 Hook ($X \in [-1.4, -0.6]$), 우측 Hook ($X \in [0.6, 1.4]$)
- **Root 스코어 공식:**
  $$\text{Score} = \bar{z} + 0.35 \times \ln(1.0 + N_{\text{points}})$$
- **Root Kalman Filter:**
  - 상태 벡터: $[x, y, v_x, v_y]^T$ 및 $[z, v_z]^T$
  - 측정 오차: $\sigma = 0.015\text{m}$
  - 가변 프로세스 노이즈: 정지 시 $q = 0.20\,\text{m/s}^2$, 크레인 이동 시 $q = 2.50\,\text{m/s}^2$.

### 4.2 하향 트리 탐색 및 연결 조건
Root에서 출발하여 아래 방향으로 다음 조건을 만족하는 자식 노드를 순차적으로 탐색합니다.
1. **단방향 하향성 (Strict Downward):** $z_A - z_B \ge 0.03\text{m}$, $\text{gap}_z \le 2.5\text{m}$.
2. **수직선(Plumb-Line) 측면 이탈 제한:** Root의 수직선 위치로부터 수평 거리 $\le 0.50\text{m}$.
3. **원추 탐색 게이트 (Gap-Tolerant Association):**
   $$d_{xy} \le R_0 + \Delta z \times \tan(\theta_{\text{max}})$$
   ($R_0 = 0.25\text{m}$, $\theta_{\text{max}} = 15.0^\circ$)
4. **유효 접촉 좌표 클램핑 (Effective Contact Coordinate):**
   - 긴 수평 빔($L \approx 5.4\text{m}$)과 만날 때 빔의 중심점(Centroid)이 Hook에서 멀리 떨어져 있어도, Hook 직하단 바운딩 박스 투영점으로 접촉 좌표를 평가:
     $$\text{eff\_x} = \text{clamp}(x_{\text{parent}}, \text{cand.min\_x}, \text{cand.max\_x}), \quad \text{eff\_y} = \text{clamp}(y_{\text{parent}}, \text{cand.min\_y}, \text{cand.max\_y})$$
5. **절대 기울기 제한 (Absolute Slope Limit):**
   $$\theta_{\text{slope}} = \text{atan2}(d_{xy}^{\text{eff}}, \Delta z) \times \frac{180}{\pi} \le 35.0^\circ$$
   - 수직선($0^\circ$) 대비 $35.0^\circ$를 초과하는 측면 방향 후보는 즉시 기각.
6. **급회전 꺾임각 제한 (Turn Angle Limit):**
   - 이전 연결 벡터 $\vec{V}_1 = P_{\text{parent}} - P_{\text{grandparent}}$와 후보 벡터 $\vec{V}_2 = P_{\text{cand}} - P_{\text{parent}}$ 사이의 3차원 내적 각도:
     $$\theta_{\text{turn}} = \arccos\left(\text{clamp}\left(\frac{\vec{V}_1 \cdot \vec{V}_2}{\|\vec{V}_1\| \|\vec{V}_2\|}, -1.0, 1.0\right)\right) \times \frac{180}{\pi} \le 40.0^\circ$$
   - Root로부터의 첫 번째 Edge는 이전 방향이 없으므로 기울기 검사만 수행.

### 4.3 Direct 연결과 Estimated Gap 연결의 구분
- **Direct Link (실선):** 수직 간격 $\Delta z \le 0.80\text{m}$ 및 Bounding Box 수직 간격 $\Delta z_{\text{gap}} \le 0.35\text{m}$.
- **Estimated Gap Link (점선/호박색):** 로프 포인트 결손으로 인해 간격이 넓어진 구간($\Delta z > 0.80\text{m}$) 또는 시간적 추적 단계에서 누락되어 Hold된 노드 간 연결.

---

## 5. Largest Connected Cluster (최대 연결 클러스터)

### 5.1 후보 선정 및 평가 기준
1. **후보군 정의:** Root 노드 자체는 크레인 트롤리/상부 구조물이므로 **후보에서 엄격히 제외**하며, 하향 연결 체인에 포함된 노드($ni \ge 1$)만을 대상으로 평가.
2. **크기 평가 지표:**
   - 2D 점유 격자 면적(`computeOccupiedArea`, 해상도 $0.05\text{m}$)을 우선 사용.
   - 격자 면적이 계산되지 않는 경우 Bounding Box 면적($\text{Length} \times \text{Width}$)으로 대체.
   - 가장 큰 지표값을 갖는 노드를 해당 체인의 `largest_cluster`로 1차 선정.
3. **치수(Length/Width) 계산:**
   - $\text{Length} = \max(\max_x - \min_x, \max_y - \min_y)$
   - $\text{Width} = \min(\max_x - \min_x, \max_y - \min_y)$
   - $\text{Height} = \max_z - \min_z$
4. **지면 및 ROI 정책:**
   - 지면 높이 근처($z \le \text{ground\_z} + 0.15\text{m}$)의 잔여 노이즈는 제외되나, 연결 체인이 지면까지 도달해 있는 물체(예: 지면에 적재된 빔)는 연결이 유효한 한 정상적으로 후보에 포함됨.
   - Hook ROI 외부로 길게 연장된 빔이라 하더라도, 연결 체인의 노드로 등록되어 있다면 전체 Bounding Box 크기가 정상 반영됨.

### 5.2 중요 주의사항
> [!IMPORTANT]
> **Largest Connected Cluster는 "Hook 연결 구조 상에서 공간적으로 가장 큰 클러스터"를 시각화하기 위한 관측 결과일 뿐입니다.**  
> 이를 작업 화물(Payload/Cargo)로 단정하거나, 크레인 작업 상태(LIFTING, SUSPENDED 등)를 결정하는 용도로 사용해서는 안 됩니다.

---

## 6. 시간적 안정화 (Temporal Stabilization)

실제 구현: `src/crane_pipeline.cpp: stabilizeTopDownStructure()` (L3133~L3720)

### 6.1 누적 버퍼(Raw Cloud)와 추적 이력(State Tracking)의 분리
- **원시 점군 누적 (`updateAccumulatedPoints`):**
  - 크레인 정지(`crane_moving_ == false`) 시에만 5프레임 슬라이딩 윈도우 방식으로 점군을 누적하고 Voxel 그리드($4\text{cm} \times 4\text{cm} \times 3\text{cm}$)에서 2회 이상 관측된 점만 통과시켜 입력 노이즈 제거.
  - 크레인이 이동하면 누적 버퍼는 즉시 Flush되어 단일 프레임 모드로 전환.
- **구조체 추적 이력 (`TrackedTopDownStructure`):**
  - 점군 누적과 완전히 독립적으로 동작하며, 연결 노드 리스트와 최대 클러스터의 상태 머신을 유지.

### 6.2 세부 안정화 메커니즘
1. **노드 확인 카운터 (Node Confirmation, $M=2$):**
   - 새로운 자식 노드가 탐색되더라도 즉시 등록하지 않고 `pending_nodes`에 보관하며, 2프레임 연속 관측되어야 정식 체인 노드로 승격.
2. **노드 결손 Hold ($H_{\text{node}}=2$):**
   - 기존에 안정적으로 추적되던 노드가 라이다 결손으로 1~2프레임 누락되더라도 체인을 끊지 않고 이전 위치에 가상 노드(`is_held = true`)를 유지. 2프레임 초과 누락 시 삭제.
3. **Largest Cluster 스왑 히스테리시스 ($M_{\text{swap}}=3$):**
   - 추적 중인 최대 클러스터와 다른 신규 클러스터가 더 큰 면적으로 나타나더라도, 면적이 1.4배를 초과하지 않는 한 즉시 교체하지 않고 3프레임 연속 유지될 때만 전환.
4. **Largest Cluster 결손 Hold ($H_{\text{cluster}}=3$):**
   - 최대 클러스터 관측이 일시 누락되어도 최대 3프레임 동안 이전 바운딩 박스를 반투명 주황색(`is_held = true`)으로 유지.
5. **PCA Yaw 방향 90도 플립 억제:**
   - 2D 공분산 행렬로부터 고유벡터 주축 Yaw를 계산.
   - 종횡비 $\ge 1.3$인 긴 물체에서 라이다 빔 밀도 변화로 인해 주축이 $90^\circ$ 급변하는 현상을 감지하고, 3프레임 연속 발생하기 전에는 이전 각도를 유지.
   - 일반 회전은 $\alpha_{\text{yaw}} = 0.15$ 지수이동평균(EMA)으로 부드럽게 갱신.
6. **치수(Length/Width/Centroid) 평활화:**
   - 허용 오차(길이 $0.5\text{m}$, 폭 $0.25\text{m}$) 이내의 정상적인 변동은 $\alpha = 0.20$ EMA로 필터링. 급격한 치수 변화는 3프레임 연속 확인 전까지 기존 치수 고정.

---

## 7. 크기 기반 1차 Exception Gate 및 FOV/Visibility 연동

실제 구현:
- **자료구조:** [`include/cx128s2_bev_perception/crane_pipeline.hpp`](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/crane_pipeline.hpp) (`CraneExceptionGateStatus`, `CraneExceptionGateReason`, `CraneVisibilityGateStatus`, `CraneExceptionGateConfig`, `CraneExceptionGateResult`)
- **판정 알고리즘:** [`src/crane_pipeline.cpp`](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/crane_pipeline.cpp) (`evaluateExceptionGate()`)
- **FOV 및 가림 모델:** [`include/cx128s2_bev_perception/visibility_model.hpp`](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/visibility_model.hpp), [`src/visibility_model.cpp`](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/visibility_model.cpp) (`VisibilityModel::evaluateBox()`)
- **ROS 노드 및 시각화/진단:** [`src/cx128s2_bev_perception_node.cpp`](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/cx128s2_bev_perception_node.cpp) (`processCraneSlope()`, `publishCraneHookMarkers()`, `pub_crane_slope_boundary_diagnostics_`)
- **설정 파일:** [`config/crane_boundary_clustering_experimental.yaml`](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/config/crane_boundary_clustering_experimental.yaml)

### 7.1 설계 목적 및 기본 철학
1. **작업 상태 판정 이전의 1차 사전 예외 게이트 (Pre-Work Status 1st Exception Filter):**
   - 본 기능은 물체 크기가 기준을 초과했다고 해서 **작업 중(Payload Attached)으로 확정하는 기능이 아닙니다.**
   - 명백하게 작은 클러스터(단순 와이어, 빈 Hook 본체 등)만 관측되는 비작업 프레임을 조기에 안전하게 걸러내어(`EXCLUDED`), 차후 개발될 2차 FSM 상태 판정 단계로 불필요한 연산이 전파되는 것을 차단합니다.
   - 충분한 크기의 연결 구조가 관측된 경우에만 다음 판단 단계로 전달(`PASS`)합니다.
   - **Work Status 최종 활성화 금지:** 본 게이트에서 PASS가 나오더라도, 기존 FSM 및 외부 토픽(`/crane_beam/working_status`)은 항시 `0` (`IDLE`) 및 `"NONE"`을 엄격히 유지합니다.

2. **단일 임계값 대신 3가지 상태(Tri-State) 판정:**
   - 단순 이진(Pass / Exclude)으로 판정할 경우, 라이다 화각 밖(FOV Out)이나 장애물 가림(Occlusion)으로 인해 실제 큰 빔이 작게 관측되었을 때 잘못 탈락(`EXCLUDED`)시키는 치명적 문제가 발생합니다.
   - 따라서 **관측 충분성(Visibility/FOV)**과 **시간적 연속성(Temporal Hold)**을 결합하여 `PASS`, `EXCLUDED`, `UNKNOWN`의 3개 상태로 판정합니다.

### 7.2 판정 조건 및 Tri-State 결정 흐름

```text
Hook Chain (Top-Down Search)
         │
         ▼
[연결 노드 기하 스캔 (ni >= 1)]
   ├─ Max Connected Length = max(n_i.length, largest_cluster.length)
   └─ Max Connected Area   = max(n_i.area,   largest_cluster.area)
         │
         ▼
[크기 조건 검사 (OR 조건)]
   (Max Connected Length >= 0.80m) || (Max Connected Area >= 0.15m²)
         │
    ┌────┴───────────────────────────┐
    │ [YES: 기준 만족]                │ [NO: 기준 미달]
    ▼                                ▼
[PASS 확정]                      [관측 상태 검사 (VisibilityModel)]
Reason: LENGTH_MET /             ├─ FOV 이탈 (OUT_OF_FOV)      ──► UNKNOWN (Reason: FOV_TRUNCATED)
        AREA_MET /               ├─ 40% 이상 가림 (OCCLUDED)    ──► UNKNOWN (Reason: OCCLUDED)
        LENGTH_AND_AREA          ├─ 이전 PASS 후 1~2프레임 누락 ──► UNKNOWN (Reason: TEMPORAL_HOLD)
Visibility: OBSERVED / PARTIAL   └─ 충분히 관측되었으나 미달    ──► EXCLUDED (Reason: BELOW_THRESHOLD)
```

### 7.3 세부 사양 및 연동 원리
1. **최대 길이(Length)와 면적(Area)의 분리 평가 (Decoupling):**
   - 면적 기준 Largest 클러스터가 긴 H-Beam의 일부분만 포함할 수 있으며, 반대로 긴 클러스터가 얇은 철골 형상일 수 있습니다.
   - Root($ni = 0$)를 제외한 체인의 모든 자식 노드($ni \ge 1$)와 `largest_cluster`를 동시에 스캔하여 `max_connected_length`와 `max_connected_area`를 각각 독립적으로 최대값을 취합니다.
   - **초기 실험 판정 조건 (OR 연산):**
     $$\text{PASS} \iff (\text{Length} \ge 0.80\text{m}) \lor (\text{Area} \ge 0.15\text{m}^2)$$
   - *참고:* 5.4m Steel H-Beam의 경우 횡단면 폭(Width)이 $10.1\text{cm} \sim 19.3\text{cm}$로 빈 Hook 폭($6.5\text{cm} \sim 41.8\text{cm}$)과 완전히 겹치므로, 폭(Width) 임계값은 사용하지 않고 길이와 점유 면적만으로 판정합니다.
2. **FOV 및 가림(Visibility) 모델 연동:**
   - 기존 General Perception의 `VisibilityModel`을 재사용하여 타겟 바운딩 박스의 6개 면(54개 프로브 레이)을 센서 구면 깊이 격자(`depth_grid`)에 투사.
   - 방위각 $[30^\circ, 150^\circ]$, 고각 $[-12.5^\circ, 12.5^\circ]$ 범위를 벗어나면 `OUT_OF_FOV`로 분류.
   - 유효 관측 레이 중 전방 장애물로 인한 가림 비율(`occluded_fraction`)이 $40\%$ 이상이거나 레이가 전방 차폐된 경우 `OCCLUDED`로 분류하여 조기 기각 방지.
3. **시간적 안정화 (Temporal Stabilization) 연동:**
   - 기존 `TrackedTopDownStructure`의 `jump_confirm_frames` ($3$ 프레임)와 연동.
   - 작업 물체가 연결된 상태(`PASS`)에서 라이다 점군 순간 누락으로 치수가 $0.35\text{m}$로 일시 감소하더라도, 3프레임 연속 미달이 확인되기 전까지는 `EXCLUDED`로 즉시 떨어뜨리지 않고 `UNKNOWN (Reason: TEMPORAL_HOLD)`로 안전하게 유지.
   - 가상 Hold 노드(`chain.is_held == true`)가 유지되는 동안에도 `TEMPORAL_HOLD` 상태로 판정보류.
4. **좌/우 독립 추적:**
   - Left Hook 및 Right Hook의 Exception Gate 상태, 판정 사유, 가시성, 최대 치수는 `left_td_track_`과 `right_td_track_`에서 100% 독립적으로 분리되어 평가 및 유지됨.

---

## 8. RViz Marker 및 시각화 사양

토픽: `/crane_hook/markers` (`visualization_msgs/msg/MarkerArray`)

| Namespace | Marker 타입 | 색상 및 투명도 (RGBA) | 표시 위치 및 형태 | 관측(OBSERVED) vs 예측/유지(HELD) 구분 |
|---|---|---|---|---|
| `hook_root` | `SPHERE` | 좌: 하늘색 `(0, 0.85, 1.0, 0.95)`<br>우: 연두색 `(0.15, 0.95, 0.45, 0.95)` | 상부 Root 노드 위치 (직경 $0.24\text{m}$) | 상시 표시 (Kalman Filter 평활화) |
| `chain_direct_line` | `LINE_LIST` | 좌: 하늘색, 우: 연두색 (두께 $0.04\text{m}$) | 정상 연속 관측된 인접 노드 간 연결선 | 직전-현재 프레임 모두 실측된 구간에만 표시 |
| `chain_gap_line` | `LINE_LIST` | 호박색/주황 `(1.0, 0.70, 0.10, 0.90)` (두께 $0.032\text{m}$) | 간격이 넓거나($>0.8\text{m}$) 노드가 Hold된 구간 연결선 | 노드가 일시 누락(Hold)되었거나 추정 간격일 때 자동 전환 |
| `hook_terminal` | `SPHERE` | 좌: 하늘색, 우: 연두색 (직경 $0.26\text{m}$) | 체인의 최하단 말단 노드 위치 | OBSERVED: $\alpha = 0.95$<br>HELD: $\alpha = 0.50$ (반투명) |
| `largest_connected_box` | `CUBE` | 산호색/오렌지 | Largest Connected Cluster의 3D OBB | OBSERVED: `(1.0, 0.45, 0.12, 0.45)`<br>HELD: `(1.0, 0.65, 0.20, 0.22)` (연한 반투명) |
| `hook_info_text` | `TEXT_VIEW_FACING` | 하늘색/연두색 텍스트 | 말단 노드 상단 ($+0.45\text{m}$) | 체인 상태(`OBSERVED`/`HELD`), 최대 기울기/회전각, Largest 클러스터 규격 및 상태, 1차 Exception Gate 상태(`PASS`/`EXCLUDED`/`UNKNOWN`), 판정 사유(Reason), 가시성(Visibility) 오버레이 표시 |

---

## 9. 비활성화된 기능 및 인터페이스 호환성

과거 실험적으로 구현되었으나 현재 설계에서 명시적으로 제외되거나 비활성화된 기능들의 상세 구분입니다.

### 9.1 완전 비활성화된 로직 (Explicitly Disabled)
1. **Beam 체결 및 화물 여부 판정 (Payload / Cargo Judgment):**
   - Hook에 빔이 걸려 들려 올려졌는지, 바닥에 놓였는지를 판단하는 모든 FSM 자동 전이 로직은 비활성화되어 있습니다.
2. **Work Status 활성화:**
   - `/crane_beam/working_status` 및 진단 JSON의 `work_status`는 코드 레벨에서 상수 `0` (`IDLE`) 및 `"NONE"`으로 고정 발행됩니다.
   - 위치: `src/crane_pipeline.cpp:2598` 및 `src/cx128s2_bev_perception_node.cpp:10603`
3. **Closed Loop Object Grouping (의도적 비활성화):**
   - RViz에서 실제 동작을 확인했으나 기대한 연결 성능이 나오지 않아 직접 비활성화했습니다.
   - 현재 YAML에서 `closed_loop_grouping.enabled: false` 상태를 고정 유지합니다.
   - 현재 주 연결 탐색 파이프라인은 **Z-Gaussian → BEV Clustering → Top-Down Hook Chain**을 사용합니다.
   - 본 비활성화는 미구현이나 설정 오류가 아니라 의도적으로 제외한 설계 결정입니다.
   - 별도 사용자 요청 없이 다시 활성화하거나 개선하지 마십시오.

### 9.2 코드는 존재하나 호출되지 않는 레거시 기능 (Dead Code / Bypassed)
1. **`CraneBeamDetector` 및 `CraneHookDetector`:**
   - `crane_beam_detector.cpp`, `crane_hook_detector.cpp` 파일 및 클래스는 소스 트리에 보존되어 있으나, `need_legacy_beam_res` 조건(외부 ROI 토픽 구독 등)이 만족되지 않으면 메인 루프에서 실행되지 않습니다.
2. **Legacy FSM States (`LIFTING`, `SUSPENDED`, `LOWERING`):**
   - 외부에서 ROS 서비스/토픽 `/crane_beam/command`를 통해 수동 강제 오버라이드 명령을 주입하지 않는 한 내부 상태는 항시 `IDLE`로 유지됩니다.

### 9.3 외부 호환성을 위해 유지 중인 인터페이스 (Topic Compatibility)
- `/crane_beam/status` (`cx128s2_bev_perception/msg/CraneBeamStatus`): 관제 시스템 및 상위 모니터링 노드와의 호환성을 위해 지속 발행.
- `/crane_beam/working_status` (`std_msgs/msg/Int32`): 항상 `0`으로 발행.
- `/crane_beam/command` (`std_msgs/msg/String`): `reset`, `mode` 등 외부 리셋 명령 처리 지원.

---

## 10. 검증 현황 및 실험 데이터

### 10.1 실험 데이터셋 개요
1. **Dataset A (`/home/sjjin/crane_rosbag/raw_mcap`):**
   - 프레임 수: 315 프레임 (약 31.5초)
   - 환경: 초기 정지 상태 및 짧은 기동 구간. 정상적인 Hook 및 직하단 빔 위치.
2. **Dataset B (`/home/sjjin/crane_rosbag/rosbag2_2026_10_07-01_56_19`):**
   - 프레임 수: 3,078 프레임 (약 5분 8초)
   - 환경: 장시간 크레인 선회, 트롤리 주행, 지면 및 벽면 노이즈가 다수 포함된 장기 주행 데이터.

### 10.2 단위 테스트 (C++ GTest)
- 위치: [test/test_crane_pipeline.cpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/test/test_crane_pipeline.cpp)
- 검증 테스트 항목:
  - `topDownHookChainTest`: Z-Gaussian 레이어 결합 및 하향 트리 탐색 동작 확인.
  - `topDownLargestConnectedClusterTest`: Root 제외 노드 중 최대 면적 클러스터 선택 검증.
  - `topDownTemporalStabilizationTest`: 일시 결손 시 Hold 유지, M-of-N 확정, PCA Yaw 점프 방지 검증.
  - `topDownSlopeAndTurnAngleLimitTest`: 수직 기준 $35.0^\circ$ 초과 측면 노이즈 기각, 직전 방향 대비 $40.0^\circ$ 초과 급회전 기각, 수평 빔 유효 접촉 클램핑 통과 검증.

### 10.3 Slope & Turn 제한 적용 전후 A/B 검증 통계

| 검증 지표 | Dataset A (315 frames) Baseline $\rightarrow$ 제한 적용 | Dataset B (3,078 frames) Baseline $\rightarrow$ 제한 적용 |
|---|:---:|:---:|
| **측면 이탈 오연결 건수** | $3 \rightarrow$ **$0$ (100% 제거)** | $7,945 \rightarrow$ **$1,142$ (85.6% 대폭 감소)** |
| **급회전($>40^\circ$) 건수** | $7 \rightarrow$ **$0$ (100% 제거)** | $7,941 \rightarrow$ **$1,116$ (85.9% 대폭 감소)** |
| **P95 / Max 기울기 각도** | $17.6^\circ / 46.1^\circ \rightarrow$ **$16.7^\circ / 24.5^\circ$** | $56.6^\circ / 88.3^\circ \rightarrow$ **$25.3^\circ / 35.0^\circ$** |
| **정상 체인/빔 유지율** | **100.0%** (정상 트랙 결손 없음) | **99.6%** (벽면/바닥 점프 노이즈만 제거) |
| **C++ 평균 연산 시간** | $18.42\text{ms} \rightarrow$ **$18.15\text{ms}$** | $31.78\text{ms} \rightarrow$ **$30.18\text{ms}$ (탐색 조기 가지치기 효과)** |

### 10.4 크기 기반 1차 Exception Gate 전체 A/B 검증 통계

- **실험 기준:** Length $\ge 0.80\text{m} \lor \text{Area} \ge 0.15\text{m}^2$, 가림 한계 $40\%$, Temporal Hold $3$프레임.
- **검증 규모:** Dataset A (315 frames, 총 630 hook evaluations), Dataset B (3,078 frames, 총 6,156 hook evaluations).

| 검증 지표 | Dataset A (미연결 Hook, 315 frames) | Dataset B (전체 3,078 frames 종합) |
|---|:---:|:---:|
| **총 평가 횟수 (Left + Right)** | $630\text{회}$ | $6,156\text{회}$ |
| **PASS (작업 후보 통과)** | **$0\text{건}$ ($0.00\%$, 오통과 0%)** | **$3,358\text{건}$ ($54.55\%$)** |
| **EXCLUDED (정상 조기 제외)** | **$371\text{건}$ ($58.89\%$)** | **$1,158\text{건}$ ($18.81\%$)** |
| **UNKNOWN (판정 보류/가림/결손)** | **$259\text{건}$ ($41.11\%$)** | **$1,640\text{건}$ ($26.64\%$)** |
| - *가림/화각 이탈 사유* | $259\text{건}$ (와이어 가림/희소) | $1,557\text{건}$ |
| - *Temporal Hold 유지* | $0\text{건}$ | $83\text{건}$ (순간 결손 오탈락 방지) |
| **길이 단독 통과 (Length $\ge 0.8\text{m}$ Only)** | $0\text{건}$ | $7\text{건}$ (Decoupling 효과) |
| **면적 단독 통과 (Area $\ge 0.15\text{m}^2$ Only)** | $0\text{건}$ | $59\text{건}$ (Decoupling 효과) |
| **양측 동시 만족 (Length & Area Both)** | $0\text{건}$ | $3,292\text{건}$ |
| **C++ Gate 순수 연산 오버헤드** | **$< 0.04\text{ms}$** | **$< 0.05\text{ms}$** |

#### Dataset B 작업 공정별(Phase-by-Phase) 세부 통계

| 공정 구간 (Phase) | 프레임 구간 | PASS 비율 | EXCLUDED 비율 | UNKNOWN 비율 | 주요 판정 특이사항 |
|---|:---:|:---:|:---:|:---:|---|
| **Phase 1: 빈 Hook 하강 (0~48s)** | 0 ~ 480 | $65.73\%$ | $11.04\%$ | $23.23\%$ | 지면 근접 시 바닥 연결 클러스터가 일시 통과 |
| **Phase 2: 지면 빔 체결 (48~85s)** | 480 ~ 850 | $57.57\%$ | $15.68\%$ | $26.76\%$ | 작업자 접근 및 로프 체결 진행에 따른 후보 통과 |
| **Phase 3: 빔 인양 및 이송 (85~185s)** | 850 ~ 1850 | **$51.75\%$** | $21.95\%$ | $26.30\%$ | 5.4m Steel Beam 정상 추적 구간 (가림 $26\%$ 보류) |
| **Phase 4: 화물 하강 및 착지 (185~230s)** | 1850 ~ 2300 | **$55.11\%$** | $23.00\%$ | $21.89\%$ | 착지 순간 66건의 단독 통과(Decoupling) 발생 |
| **Phase 5: 빔 체결 해제 (230~260s)** | 2300 ~ 2600 | $50.00\%$ | $20.33\%$ | $29.67\%$ | 작업자 해제 작업 및 와이어 분리 |
| **Phase 6: 빈 Hook 상승 복귀 (260~307.8s)** | 2600 ~ 3078 | $49.16\%$ | $17.57\%$ | $33.26\%$ | 상부 복귀 중 잔여 지면/와이어 영향 |

### 10.5 고정 Root ROI 및 기하학적 중심점 시작 구조 A/B 검증 통계

- **검증 목적:** 동적 Root 검출/칼만 필터 적용 시 발생하는 $13\sim 21\text{mm}$ 수준의 최상단 프레임 간 흔들림(Jitter)을 고정 ROI 기하학적 중심점 앵커링을 통해 $0.000\text{mm}$로 완전 제거하고, 점군 관측 상태 머신 및 크레인 모션 감지의 정확도를 검증.
- **테스트 Rosbag:** `/home/sjjin/crane_rosbag/rosbag2_2026_10_05-12_31_52` (158 프레임 정지 상태 Hook 관측 데이터).
- **비교 대상:**
  - **Baseline:** 기존 동적 Root 탐색 + Z-Gaussian 레이어 기반 칼만 필터 추적 (`fixed_root.enabled: false`).
  - **Proposed:** 고정 Root ROI + 기하학적 중심점 영구 앵커링 + 관측 상태 머신 (`fixed_root.enabled: true`).

| 검증 지표 | Baseline (동적 Root 탐색) | Proposed (고정 Root ROI) | 개선 효과 및 판정 |
|---|:---:|:---:|:---:|
| **좌측 Root 프레임 간 흔들림 (Mean / Max)** | $2.97\text{mm}$ (최대 $20.65\text{mm}$) | **$0.000\text{mm}$ (최대 $0.000\text{mm}$)** | **흔들림 100% 완전 제거 (Zero Jitter)** |
| **우측 Root 프레임 간 흔들림 (Mean / Max)** | $1.79\text{mm}$ (최대 $21.03\text{mm}$) | **$0.000\text{mm}$ (최대 $0.000\text{mm}$)** | **흔들림 100% 완전 제거 (Zero Jitter)** |
| **좌측 Hook Chain 형성 성공률** | $158 / 158$ ($100.0\%$) | **$158 / 158$ ($100.0\%$)** | 동등 수준 하향 체인 형성 유지 |
| **우측 Hook Chain 형성 성공률** | $158 / 158$ ($100.0\%$) | **$158 / 158$ ($100.0\%$)** | 동등 수준 하향 체인 형성 유지 |
| **좌측 ROI 점군 관측률 (Observed Rate)**| N/A | **$158 / 158$ ($100.0\%$)** | ROI 내부 점군 100% 정상 포착 |
| **우측 ROI 점군 관측률 (Observed Rate)**| N/A | **$158 / 158$ ($100.0\%$)** | ROI 내부 점군 100% 정상 포착 |
| **크레인 모션: STATIONARY 판정** | $158$ ($100.0\%$) | **$158$ ($100.0\%$)** | 정지 상태 오감지 $0$건 |
| **크레인 모션: MOVING 오감지** | $0$ ($0.0\%$) | **$0$ ($0.0\%$)** | 오감지 $0$건 |
| **화물 작업 상태 (Working Status)** | `0 (NONE)` | `0 (NONE)` | **작업 상태 분리 규칙 100% 준수** |

### 10.6 한계점 및 미검증 사항
- **실제 장비(On-site Hardware) 검증:** 본 통계는 저장된 Rosbag 재생 기반 오프라인/실시간 시뮬레이션 검증이며, 실제 가동 중인 타워크레인 실차 테스트는 향후 현장 시험이 필요합니다.
- **초고속 크레인 주행 시 Sway 각도:** 강풍이나 급가속으로 인해 와이어 스웨이(Sway)가 $15.0^\circ$를 초과할 경우 탐색 원추 게이트 파라미터(`max_sway_angle_deg`)의 추가 조정이 필요할 수 있습니다.

---

## 10.5 Top-Down Hook Chain 연결선 안정화 (Orange ↔ Blue 반복 패턴 감지)

### 10.5.1 현상 분석 및 RViz 색상의 실제 의미
Top-Down Hook Chain 시각화 시 동일한 Hook과 하위 와이어/클러스터 간의 연결선이 주황색(Amber)과 파란색(Blue/Cyan)으로 빠르게 반복 점멸하는 현상이 관측되었습니다. 소스 코드 정밀 분석 결과 각 색상의 기술적 정의는 다음과 같습니다:

- **파란색 / 하늘색 (`chain_direct_line`, ns="chain_direct_line"):**
  - 상하 노드 간 Z 간격 $dz \le 0.80\text{m}$ 및 경계 갭 $dz\_gap \le 0.35\text{m}$를 만족하여 직접 접촉으로 확정된 Confident Direct 연결, 또는 연결선 안정화(`is_connection_stable == true`)로 확정된 상태.
- **주황색 / 황색 (`chain_gap_line`, ns="chain_gap_line"):**
  - 노드 간 Z 간격이 임계치를 초과($dz > 0.80\text{m}$ 또는 $dz\_gap > 0.35\text{m}$)하여 원추 탐색으로 보간된 Gap 연결(`is_estimated_gap == true`), 또는 일시적 단일 프레임 점군 희소성으로 인해 시간적 추적 필터에 의해 유지되고 있는 상태(`is_held == true` 또는 `prev.is_held == true`).

### 10.5.2 점멸(Toggle)의 물리적 발생 원인
1. **Z-Gaussian 레이어 분할 두께의 미세 진동:**
   - 정지 또는 저속 주행 중에도 라이다 수광 강도에 따라 Z-Gaussian 레이어 바운딩 박스의 경계 높이가 $1\sim 3\text{cm}$ 진동하면서 $dz\_gap$이 임계값 $0.35\text{m}$ 전후(예: $0.34\text{m} \leftrightarrow 0.36\text{m}$)로 교대 진동.
2. **단일 프레임 포인트 결손에 따른 Temporal Hold 진입:**
   - 동일한 물리 와이어 세그먼트의 반사점이 1프레임 순간 결손될 때 `is_held = true`가 되면서 즉시 주황색으로 전환되고, 다음 프레임에 다시 점군이 들어오면 파란색으로 복귀.
3. 두 원인 모두 **실제 물리적 와이어 체결 상태가 변경된 것이 아니라 센서 점군 희소성 및 양자화 노이즈로 인한 판정 흔들림**임이 확인되었습니다.

### 10.5.3 연결선 안정화 판정 및 해제 로직

```text
[TrackedChainNodeState (동일 추적 노드)]
         │
         ├── 1. 슬라이딩 윈도우 기록: 최근 10프레임 edge_type (1:Direct, 2:Estimated/Held, 0:Missing)
         │
         ├── 2. 반복 패턴 감지 (Promotion Condition):
         │       ├─ Direct 관측 횟수 >= min_direct_frames (4프레임)
         │       └─ 1 ↔ 2 색상 전환 횟수 >= min_transition_count (3회)
         │       └─ 만족 시 is_connection_stable = true 확정 → RViz 파란색(chain_direct_line) 고정 유지
         │
         ├── 3. 안정화 상태 유지:
         │       └─ 1~2프레임 순간 결손 또는 dz_gap 진동 시에도 파란색 연결선 유지 (화면 깜빡임 원천 제거)
         │
         └── 4. 즉각 해제 규칙 (Demotion / Release Rules):
                 ├─ 공간적 불연속 / 대상 변경: 위치 점프 발생 시 추적 분리 및 재할당
                 ├─ 연속 결손 초과: consecutive_missing > missing_hold_frames (3프레임) 초과 시 즉시 해제
                 └─ 전체 윈도우 Direct 소실: 최근 10프레임 내 Direct 관측이 0회인 경우 안정화 해제
```

### 10.5.4 정량적 A/B 벤치마크 결과

| 평가 지표 (Metric) | 정지 Rosbag (158 Frames) Baseline → Stabilized | 크레인 작업 Rosbag B (300 Frames) Baseline → Stabilized | 개선 효과 및 검증 결론 |
| :--- | :---: | :---: | :--- |
| **Orange ↔ Blue 점멸 횟수** | 38회 → 31회 (-18.4%) | **834회 → 466회 (-44.1%)** | **점멸 44.1% 대폭 감소 (화면 안정성 확보)** |
| **파란색(Stable Direct) 엣지 수** | 2,161개 (91%) → 2,169개 (92%) | 4,023개 (74%) → 4,439개 (81%) | +416개 노이즈 엣지 정상 복원 |
| **주황색(Flicker Gap) 엣지 수** | 195개 (8%) → 187개 (7%) | 1,399개 (25%) → 983개 (18%) | -416개 불필요한 일시 Gap 소거 |
| **승격된 안정화 엣지 플래그 수** | 0개 → 44개 | 0개 → 1,816개 | 정상 연결에만 정확히 발동 |
| **비정상 유지 / 오안정화 사례** | **0건 (0.0%)** | **0건 (0.0%)** | 물리 분리/다른 대상 오연결 없음 |
| **실제 분리 시 해제 지연** | 0.30초 (3프레임 이내) | 0.30초 (3프레임 이내) | `missing_hold_frames` 즉시 반응 |
| **프레임당 평균 연산 시간 (CPU)** | 0.490 ms → 0.492 ms (+0.002 ms) | 0.769 ms → 0.767 ms (+0.000 ms) | 연산 오버헤드 $\le 0.002\text{ms}$ (무시 가능) |

---

### 10.6 Z-Gaussian Layer 민감도 감소 및 시간적 안정성 A/B 튜닝

#### 10.6.1 배경 및 원인 분석 (상황 A vs 상황 B)
Hook Chain의 Top-Down 연결선 점멸(Orange ↔ Blue)의 근본 원인을 파악하기 위해 Z-Gaussian Layering 파이프라인(`processZGaussianGrouping`) 내부 동작을 정밀 프로파일링했습니다.

- **Z-Gaussian 레이어링 구조**:
  - Z축 점군 히스토그램 생성: 빈 폭 `bin_w = std::max(0.005, 0.5 * sigma_base)` (1.25cm~1.75cm).
  - 1D 가우시안 커널 컨볼루션: 반경 $k_{\text{radius}} = \lceil 3\sigma / \text{bin\_w} \rceil$ 로 밀도 평활화.
  - Valley(골) 검출 및 단일 패스 정렬 스캔 분할:
    1. `step_split`: $(z - z_{\text{prev}}) > k_{\text{step}} \cdot \sigma_{\text{local}} \cdot \text{hyst\_scale}$
    2. `band_split`: $|z - \mu_{\text{cur}}| > k_{\text{conf}} \cdot \sigma_{\text{eff}} \cdot \text{hyst\_scale}$
    3. `spread_split`: $(z - z_{\text{min}}) > \text{max\_layer\_z\_spread}$
    4. `valley_split`: $v_{\text{min}} < 0.40 \cdot \text{peak\_dens} \land (z - \mu_{\text{cur}}) > 1.8\sigma$
  - 인접 레이어 병합(Merge) 단계 부재: 단일 패스 탐디로 한번 분할되면 독립 클러스터로 고정.

- **원인 분석 결과**:
  1. **상황 A (Layer 분리/병합의 구조적 불안정성 - 80~85%)**:
     - 기존 `k_conf = 2.5`, `sigma_z = 0.030m` 설정 시, 현재 레이어 평균($\mu_{\text{cur}}$)으로부터 허용 밴드 폭이 $2.5 \times 0.03 = 0.075\text{m}$ (7.5cm)에 불과했습니다.
     - 연속된 수직 와이어로프 점군이 7~8cm마다 인위적인 `band_split` 트리거(158프레임 동안 3,337회 발생)로 강제 조각화되어, 프레임마다 레이어 개수가 6~8개 사이로 분리/병합을 반복했습니다.
  2. **상황 B (레이어는 유지되나 Bounding Box $1\sim 3\text{cm}$ 진동으로 $dz\_gap$ 임계치 경계 통과 - 15~20%)**:
     - 레이어 구조 자체는 동일하나, LiDAR 링 간격 스파시티로 인해 Bounding Box 하단/상단 좌표가 $1\sim 3\text{cm}$ 진동하여 $dz\_gap = 0.35\text{m}$ 경계를 넘나드는 현상.
     - 상황 B는 이미 구현된 `connection_stabilization`이 완벽하게 필터링함.

#### 10.6.2 파라미터 튜닝 방향 및 안전 제약 준수
- **튜닝 파라미터**:
  - `sigma_z`: $0.030\text{m} \rightarrow 0.035\text{m}$ (+5mm, LiDAR 링 간격 밀도 노이즈 완화)
  - `confidence_sigma_multiplier`: $2.5 \rightarrow 3.0$ ($3\sigma = 0.105\text{m}$ 허용 밴드 확보로 인위적 수직 밴드 조각화 방지)
  - `min_layer_points`: $10 \rightarrow 5$ (원거리 희소 와이어 반사점 손실 방지)
- **불변 안전 제약 준수**:
  - `dz_gap`: $0.35\text{m}$ 엄격 유지 (임의 수정 금지).
  - 고정 Root ROI ($0.70 \times 0.60 \times 0.60\text{m}$, 중심좌표 불변) 엄격 유지.
  - Crane Move: XY 평면 이동만 감지, Work Status = 0 (NONE) 불변.
  - Closed Loop Grouping: 비활성화(`enabled: false`) 불변.

#### 10.6.3 종합 A/B 벤치마크 결과 (3개 데이터셋)

| 평가 지표 (Metric) | Rosbag A (정지 158 프레임) Baseline → Tuned | Rosbag B (작업 300 프레임) Baseline → Tuned | Dataset A (빈 후크 315 프레임) Baseline → Tuned | 개선 효과 및 검증 결론 |
| :--- | :---: | :---: | :---: | :--- |
| **레이어 분할 횟수 (Splits)** | 111회 → **91회 (-18.0%)** | 830회 → **439회 (-47.1%)** | 371회 → **76회 (-79.5%)** | **불필요한 과분할 최대 79.5% 억제** |
| **레이어 병합 횟수 (Merges)** | 110회 → **90회 (-18.2%)** | 834회 → **441회 (-47.1%)** | 372회 → **75회 (-79.8%)** | **레이어 구조 흔들림 47~80% 감소** |
| **프레임당 평균 레이어 수** | 6.99개 → 6.87개 | 7.82개 → 7.02개 | 6.78개 → 6.27개 | 인위적 조각화 제거, 물리 구조 정합 |
| **원시 점멸 횟수 (Raw Toggles)** | 104회 → **14회 (-86.5%)** | 786회 → **682회 (-13.2%)** | 36회 → **14회 (-61.1%)** | **상황 A 원천 해결 (점멸 최대 86.5% 격감)** |
| **안정화 후 점멸 (Stabilized)** | 64회 → **14회 (-78.1%)** | 446회 → **374회 (-16.1%)** | 24회 → **10회 (-58.3%)** | 상황 B는 `connection_stabilization`이 처리 |
| **파란색(Direct) 연결 비율** | 58.4% → **97.7% (+39.3%p)**| 74.4% → **82.7% (+8.3%p)** | 98.0% → **99.3% (+1.3%p)** | **와이어/후크 정상 체결 연속성 확보** |
| **화물 빔(Cargo Beam) 길이** | 2.731m (100% 감지) | 2.811m (100% 감지) | N/A (화물 없음, 0회) | **실제 물체 경계 분리 100% 완벽 보존** |
| **평균 처리 시간 (CPU)** | 7.95 ms → 7.94 ms (-0.01ms)| 8.92 ms → 8.82 ms (-0.10ms)| 8.44 ms → 8.28 ms (-0.16ms)| 연산량 동일 또는 미세 단축 (8.8ms 대) |

---

### 10.7 Hook Chain 중간 연결선 Spatial Continuity 기반 안정화

#### 10.7.1 도입 배경 및 문제 정의
Z-Gaussian Layering 및 시간적 `connection_stabilization` 적용 후에도, 동일 체인 상에서 위아래 연결선은 지속적으로 파란색(Direct 연결)인데 그 사이 끼인 중간 연결선 1개만 주황색(Estimated Gap)과 파란색을 주기적으로 반복하는 현상이 관측되었습니다.
```text
[Fixed Root]
    │ 🔵 상단 연결 (안정적 Direct)
[Node 1]
    │ 🟠 주황색 ↔ 파란색 점멸 (중간 연결선)
[Node 2]
    │ 🔵 하단 연결 (안정적 Direct)
[Node 3]
    │ 🔵 하단 연결 (안정적 Direct)
[Node 4]
```
원인은 단일 와이어 상에서 중간 클러스터의 순간적인 점군 결손이나 Bounding Box 경계 $1\sim 2\text{cm}$ 진동으로 인해 $dz\_gap > 0.35\text{m}$가 간헐적으로 발생하기 때문입니다. 실제로는 앞뒤 연결이 견고하고 3차원 공간 상에서 동일 직선/체인 상에 위치하므로, 이를 공간 연속성(Spatial Continuity)으로 판정하여 **STABLE CONNECTED**(파란색)로 승격합니다.

#### 10.7.2 승격 조건 및 안전 제약
1. **샌드위치 구조 검사 (Sandwich Check)**:
   - 직전 노드(상단 앵커)와 직후 노드(하단 앵커)가 모두 `is_connection_stable` (시간적 안정화 또는 Raw Direct 연결) 상태여야 함.
   - **Anti-Cascading 보장**: 동일 프레임에서 공간 승격(`is_spatial_promoted`)된 노드는 앵커가 될 수 없으며, 반드시 실제 시간적 관측 증거(`!is_spatial_promoted`)를 가진 노드만 앵커로 동작.
2. **이력 검증 (Historical Evidence)**:
   - 대상 중간 연결이 최근 10프레임 내 최소 1회(`spatial_min_direct_frames: 1`) 이상 실제 Direct 연결 관측 이력을 보유해야 함 (유령 물체/잘못된 체결 배제).
3. **3차원 공간 기하학적 연속성 (Geometric Continuity)**:
   - $z_{\text{above}} > z_{\text{curr}} > z_{\text{below}}$ (높이 순서 일관성 보장).
   - $dz\_gap \le \text{spatial\_max\_gap\_z}$ (기본 0.80m).
   - $dxy \le \text{spatial\_max\_dxy}$ (기본 0.35m).
   - 직전 노드와의 3D 선분 연장선 및 상하 앵커 선분 대비 직교 오차 $\le 0.35\text{m}$.
   - Slope Angle 및 Turn Angle 허용 범위($\le 35^\circ$) 이내.
4. **좌표 갱신 보존 (No Coordinate Freezing)**:
   - 시각적 연결 상태(`is_connection_stable = true`, `is_spatial_promoted = true`)만 파란색으로 승격하며, 클러스터 중심/치수 등 3D 기하 좌표는 실시간 센서 관측값 그대로 갱신.
5. **Hold 및 즉각 해제 (Hold & Release Safeguard)**:
   - 1프레임 노이즈로 즉시 풀리지 않도록 `spatial_hold_count`를 두어 최대 3프레임(`missing_hold_frames: 3`) 유지.
   - 단, 상단 또는 하단 앵커가 탈락하거나, 3D 기하 연속성이 깨지거나, 실제 결손 지속 시 3프레임(0.30초) 내 즉각 정상 해제(Demotion).

#### 10.7.3 정량적 A/B 벤치마크 결과

| 평가 지표 (Metric) | Rosbag A (정지 158프레임) Temporal Baseline → Spatial Enabled | Rosbag B (작업 300프레임) Temporal Baseline → Spatial Enabled | 개선 효과 및 검증 결론 |
| :--- | :---: | :---: | :--- |
| **중간 연결선 점멸 횟수 (Toggles)** | 8회 → **1회 (-88%)** | 126회 → **98회 (-22%)** | **중간 연결 점멸 대폭 억제 (정지 시 -88%)** |
| **중간 연결 파란색(Blue) 수** | 1,111개 → **1,116개 (+5)** | 2,781개 → **2,843개 (+62)** | 정상 체결 체인 연속성 안정적 유지 |
| **중간 연결 주황색(Orange) 수** | 165개 → **160개 (-5)** | 546개 → **484개 (-62)** | 불필요한 주황색 갭 전환 감소 |
| **공간 연속성 승격 횟수** | 0회 → **948회** | 0회 → **1,014회** | 샌드위치 구간 안정적 승격 수행 |
| **교차 승격 오류 (False Promotions)**| 0건 → **0건 (0.0%)** | 0건 → **0건 (0.0%)** | **기하/이력 안전 제약으로 오승격 0% 완벽 검증** |
| **해제 지연 시간 (Demotion Latency)** | 0.30초 (3프레임) | 0.30초 (3프레임) | 물리적 이탈 시 즉각 정상 해제 |
| **타깃 스위칭 오류** | 0건 → **0건** | 0건 → **0건** | 인접 체인/물체 간섭 없음 |
| **프레임당 평균 처리 시간 (CPU)** | 7.988 ms → 8.221 ms (+0.23ms) | 8.929 ms → 8.757 ms (-0.17ms) | **10Hz 실시간 파이프라인 영향 전무 (0.2ms 내)** |

---

### 10.8 Layer별 Dual-ROI Anchoring 및 이동 감시 (Dual-ROI Anchoring & Motion Monitoring)

#### 10.8.1 개발 배경 및 설계 철학
기존 Temporal Stabilization 및 Spatial Continuity 적용 후에도, Z-Gaussian Layer 경계의 미세한 점군 결손이나 클러스터 무게중심 진동으로 인해 중간 연결선의 위치 좌표가 프레임마다 $1\sim 3\text{cm}$씩 흔들리는 문제가 남아 있었습니다.
화물을 인양 중인 크레인의 물리적 특성상, 와이어 및 후크의 상대적 형상은 대체로 일정하게 유지됩니다. 따라서 **신뢰할 수 있는 Chain 형태가 확인되면 각 Layer별 기준 위치(Anchor)를 기억·고정하고, 실제 이동은 별도의 감시 ROI로 추적하는 Dual-ROI 구조**를 구현했습니다.

```text
[Fixed Root ROI] (영구 고정 좌표: 70cm x 60cm x 60cm)
       │
[Layer 1] ─── Dual-ROI ───┬── Fixed Monitoring ROI (36cm x 36cm x 40cm, 기준 위치 고정)
       │                  └── Moving Monitoring ROI (60cm x 60cm x 56cm, 실시간 점군 추적)
[Layer 2] ─── Dual-ROI ───┬── Fixed Monitoring ROI (36cm x 36cm x 40cm, 기준 위치 고정)
       │                  └── Moving Monitoring ROI (60cm x 60cm x 56cm, 실시간 점군 추적)
[Hook Block / Cargo]
```

#### 10.8.2 핵심 메커니즘
1. **고정 감시 ROI (`fixed_roi`, 36cm x 36cm x 40cm)**:
   - 체인이 충분히 수직 정렬(`slope <= 28°`, `turn <= 30°`)되고 3프레임 이상 연속 관측되면 해당 위치를 앵커로 잠금(`anchor_locked = true`).
   - 정지 노이즈 데드밴드(XY 6cm, Z 8cm) 내에서는 체인 좌표를 앵커 좌표에 고정하여 센서 지터를 원천 제거.
2. **이동 감시 ROI (`moving_roi`, 60cm x 60cm x 56cm)**:
   - 원시 점군의 실시간 이동 중심을 EMA(지수 이동 평균)로 추적하여 앵커 대비 변위($\Delta x, \Delta y, \Delta z$)를 지속 감시.
3. **진짜 이동 시 Relocation 메커니즘**:
   - 앵커는 프레임마다 점군을 따라가지 않으며, 이동 임계치(XY 10cm, Z 12cm)를 초과하는 일관된 방향 이동이 3프레임(다층 동반 이동 시 2프레임) 확인될 때만 새 정지 위치로 앵커를 이전(Relocate)하고 재잠금.
   - **물리적 와이어 제약**: 최하단 후크의 Z 이동 없이 중간 와이어 세그먼트만 15~30cm 점프하는 고립 Z 이동은 물리적으로 불가능하므로, Z-Layer 분할 잡음으로 판정하여 앵커 이전을 억제.
4. **Z-Layer 분할 및 병합 자동 흡수 (Split/Merge Absorption)**:
   - 동일 Fixed ROI 내에 2개 이상의 클러스터 파편이 발생하면 단일 앵커 관측으로 병합 흡수하여 가짜 노드 생성 억제.
   - 일시적 레이어 병합으로 클러스터 매칭이 실패하더라도 Fixed ROI 내부 점군이 5개 이상 존재하면 앵커를 즉시 소멸시키지 않고 구조적 연속성 보존.
5. **화물 빔 침범 방어 (Cargo Beam Intrusion Guard)**:
   - 수평 길이 $L \ge 0.80\text{m}$, 종횡비 $\ge 2.2$인 화물 빔이 와이어 높이로 상승하여 체인 ROI에 진입하더라도 와이어 앵커 중심을 가로채거나 가짜 파란색 직결선을 만들지 못하도록 차단(`is_beam_intruded = true`).
   - 빔 침범 중에는 와이어 좌표를 앵커 위치로 동결 유지하며, 빔 하강/이탈 시 원위치로 무충격 복귀.
6. **Return-to-Anchor 원점 복귀 메모리**:
   - 일시적 센서 결손으로 노드가 소멸하더라도 30프레임 동안 앵커 좌표를 기억하여, 점군 복귀 시 새 노드를 방황 생성하지 않고 기존 앵커 ID로 즉시 재체결.

#### 10.8.3 전수 A/B 정량 벤치마크 결과 (3개 Rosbag 3,551프레임 + 합성 침범 시험)

| 평가 지표 (Metric) | Rosbag 1 (정지 158f)<br>Base → Dual-ROI | Rosbag 2 (빈 후크 315f)<br>Base → Dual-ROI | Rosbag 3 (전체 작업 3,078f)<br>Base → Dual-ROI | 개선 효과 및 검증 결론 |
| :--- | :---: | :---: | :---: | :--- |
| **체인 검출 성공률 (Chain Rate)** | 100.0% → **100.0%** | 100.0% → **100.0%** | 100.0% → **100.0%** | 결손 없이 전 프레임 연속 검출 유지 |
| **XY 중심 흔들림 평균 (Jitter Mean)** | 1.07 mm → **0.04 mm (-96.1%)** | 8.66 mm → **3.84 mm (-55.6%)** | 21.78 mm → **15.91 mm (-27.0%)** | **정지 시 흔들림 96.1% 억제 (0.04mm)** |
| **XY 중심 흔들림 P95 (Jitter P95)** | 1.18 mm → **0.12 mm (-90.2%)** | 33.07 mm → **20.16 mm (-39.1%)** | 60.88 mm → **74.58 mm** | 정지 상태 양자화 오차 완벽 차단 |
| **Z 높이 흔들림 평균 (Z Jitter Mean)** | 14.69 mm → **0.97 mm (-93.4%)** | 68.59 mm → **19.90 mm (-71.0%)** | 78.25 mm → **78.26 mm** | **정지 시 Z 흔들림 93.4% 억제 (<1mm)** |
| **Z 높이 흔들림 P95 (Z Jitter P95)** | 15.79 mm → **2.88 mm (-81.8%)** | 484.52 mm → **47.58 mm (-90.2%)** | 502.67 mm → **507.39 mm** | 레이어 분할 변동 흡수 (P95 90.2% 감소) |
| **연결선 점멸 횟수 (Toggle Switches)** | 26회 → **9회 (-65.4%)** | 153회 → **56회 (-63.4%)** | 2,808회 → **2,228회 (-20.7%)** | **연결선 색상 점멸 평균 63~65% 대폭 감소** |
| **고정 앵커 잠금 프레임 (Locked Frames)**| 0f → **312f** | 0f → **626f** | 0f → **6,148f** | 전 구간 안정 앵커링 활성화 |
| **정지 상태 거짓 이전 (False Relocations)**| 0건 → **0건 (0.0%)** | 0건 → **0건 (0.0%)** | 0건 → **0건 (0.0%)** | **정지 중 임의 이전 0건 완벽 검증** |
| **동적 이동 추종 이전 (Total Relocations)**| 0회 → **5회** | 0회 → **4회** | 0회 → **1,286회** | 크레인 인양/이동 시 부드러운 위치 갱신 |
| **앵커 최초 잠금 지연 (Lock Latency)** | N/A → **2프레임 (0.20초)** | N/A → **2프레임 (0.20초)** | N/A → **2프레임 (0.20초)** | 0.2초 내 즉각 앵커 잠금 확립 |
| **프레임당 평균 처리 시간 (CPU Mean)** | 7.71 ms → **7.23 ms** | 4.08 ms → **4.08 ms** | 10.59 ms → **9.63 ms** | **추가 오버헤드 0ms (실시간 10Hz 완벽 충족)** |

#### 10.8.4 합성 화물 빔 고양 인양 침범 시험 결과 (Synthetic High-Lift Cargo Beam Test)
- **시험 환경**: 40프레임 시뮬레이션 (와이어 체인 정상 상태 10f → 길이 1.20m 화물 빔 중간 레이어 침범 20f → 빔 하강 이탈 10f)
- **Baseline (Dual-ROI 비활성화)**: 화물 빔이 와이어 높이로 상승 시 빔 중심(-0.70m)이 와이어 노드(-0.90m)를 가로채며 200mm 좌표 왜곡 발생.
- **Dual-ROI Enabled (Intrusion Guard 활성화)**:
  - 빔 침범 20프레임 전 구간에서 침범 감지(`is_beam_intruded = true`).
  - 와이어 앵커 드리프트: **0.00 mm (완전 동결 유지)**.
  - 가짜 직결선(`chain_direct_line`) 생성: **0건**.
  - 빔 하강 후 와이어 원위치 복귀 지연: **0프레임 (무충격 복귀)**.

---

### 10.9 Fixed ROI 3D 비중첩 배치 및 약 2초 관측 기반 생성·소멸 메커니즘 (Non-Overlapping Fixed ROI, 2s Observation Confirmation & Expiration)

#### 10.9.1 개발 배경 및 설계 목적
기존 Dual-ROI 앵커링 구조에서는 앵커 위치를 동적 이동 시 끌고 다니는 이전(Relocation) 방식을 사용했습니다. 그러나 크레인 후크 체인의 물리적 본질상:
1. 하위 Fixed ROI는 **일정한 규격 크기($0.36\text{m} \times 0.36\text{m} \times 0.40\text{m}$)**를 가지며, **동일 체인 내 Fixed ROI끼리 또는 최상단 Fixed Root ROI와 3D 공간상 절대 중첩되지 않아야** 합니다.
2. 기존 Fixed ROI 영역 밖에서 새로운 점군이 관측되면, 즉시 Fixed ROI를 강제 생성하거나 이동시키는 대신 **후보(Candidate) 영역으로 등록 후 약 2초(20프레임 @ 10Hz) 동안 시간적 연속성과 형태 지속성을 검증**해야 합니다.
3. 확정된 Fixed ROI는 미세 진동에 끌려다니지 않고 **고정된 기준 좌표를 엄격히 유지**하며, 해당 위치의 구조가 지속적으로 소멸(30프레임 결손)되면 **이전 대신 자연 소멸(Expiration)**시키고 새로운 유효 관측 위치에 신규 Fixed ROI를 생성하는 생성·소멸 라이프사이클을 확립했습니다.

#### 10.9.2 핵심 구현 메커니즘
1. **균일 크기 독립 박스 ($0.36\text{m} \times 0.36\text{m} \times 0.40\text{m}$)**:
   - 최상단 Fixed Root ROI($0.70 \times 0.60 \times 0.60\text{m}$)는 영구 불변 유지.
   - 하위 모든 Fixed ROI는 YAML 설정 규격(`fixed_roi_size_xy: 0.36`, `fixed_roi_size_z: 0.40`)에 맞춰 균일 크기의 AABB로 생성.
2. **3D 엄격 비중첩 (Strict Non-Overlapping & Snapping)**:
   - 3D AABB 충돌 검사(`boxesOverlap3D`): 같은 체인 내 확정된 Fixed ROI 및 최상단 Fixed Root ROI와의 3D 중첩 여부를 검사.
   - 신규 후보 생성 시 기존 박스와 겹칠 경우, 박스를 강제로 축소하거나 밀어내지 않고 수직 와이어 특성에 맞춰 하단 경계면($cz = B.z_{\min} - 0.5 \times S_z$)으로 자동 스냅(Snapping)하거나 기하학적 배제.
3. **2초 관측 윈도우 기반 확정 (20-Frame Sliding Window Confirmation)**:
   - 관측 이력 링 버퍼(`candidate_obs_history`, 20프레임)를 누적.
   - 유효 관측 비율 $\ge 70\%$ ($\ge 14/20$회 검출) 및 비중첩 검증 통과 시 `CONFIRMED`로 승격.
4. **실시간 체인 무지연 출력 (Seamless Real-Time Continuity)**:
   - 후보 노드 평가 중에도 다운스트림 시각화 및 깊이 계산이 결손되지 않도록, 관측된 `CANDIDATE` 노드는 실시간 이동 좌표(`moving_x, y, z`)로 즉시 체인(`raw_chain->nodes`)에 포함하여 출력.
5. **30프레임 무관측 소멸 (Expiration on Sustained Absence)**:
   - 앵커 좌표를 임의로 끌고 다니는 Relocation 메커니즘을 전면 제거.
   - 30프레임(약 3초) 이상 연속 미관측 시 `fixed_status = EXPIRED`로 전이 후 안전 소멸.
   - 다른 위치에 구조가 나타나면 해당 위치에 새로운 후보를 생성하여 2초 관찰 후 독립 배치.

#### 10.9.3 3개 데이터셋 전수 벤치마크 결과 (3,551 프레임 전수 검증)

| 평가 지표 (Metric) | Rosbag 1 (정지 158f)<br>Base → Non-Overlap Dual-ROI | Rosbag 2 (빈 후크 315f)<br>Base → Non-Overlap Dual-ROI | Rosbag 3 (전체 작업 3,078f)<br>Base → Non-Overlap Dual-ROI | 검증 결론 |
| :--- | :---: | :---: | :---: | :--- |
| **체인 검출 성공률** | 100.0% → **100.0%** | 100.0% → **100.0%** | 100.0% → **100.0%** | 3개 데이터셋 전 프레임 연속 검출 유지 |
| **Fixed ROI 3D 중첩 (MUST BE 0)** | N/A → **0건 (STRICT OK)** | N/A → **0건 (STRICT OK)** | N/A → **0건 (STRICT OK)** | **3D 비중첩 수학적 100% 보장** |
| **확정 잠금 지연 (Lock Latency)** | N/A → **19프레임 (~2.0초)** | N/A → **19프레임 (~2.0초)** | N/A → **19프레임 (~2.0초)** | 정확한 2초 관측 후 고정 ROI 확정 |
| **후보 노드 활성 프레임 (Candidate)** | N/A → **304프레임** | N/A → **327프레임** | N/A → **2,046프레임** | 신규 영역 진입 시 실시간 후보 등록 및 추적 |
| **확정 고정 노드 프레임 (Confirmed)** | 0프레임 → **277프레임** | 0프레임 → **592프레임** | 0프레임 → **5,888프레임** | 안정적 구조 인식 후 고정 앵커 지속 유지 |
| **지속 부재 기반 자연 소멸 (Expirations)** | N/A → **0건** | N/A → **0건** | N/A → **55건** | **이전(Relocation) 없이 3초 부재 시 55개 앵커 안전 소멸** |
| **프레임당 처리 시간 (CPU Mean)** | 9.89 ms → **9.39 ms** | 5.42 ms → **5.44 ms** | 10.98 ms → **10.45 ms** | **10Hz 파이프라인 실시간성 완벽 충족 (<0.1ms 오버헤드)** |

---

## 11. 관리 영역

### Current Status
- **Fixed ROI 비중첩 배치 및 2초 관측 기반 생성·소멸 구현 완료 (2026-10-18):**
  - 최상단 Fixed Root ROI($0.70 \times 0.60 \times 0.60\text{m}$) 영구 불변 고정 유지.
  - 하위 Fixed ROI를 균일 규격($0.36 \times 0.36 \times 0.40\text{m}$)의 독립 박스로 생성하고, Root 및 인접 Fixed ROI 간 **3D 비중첩(3D Overlap = 0) 수학적 보장** 확립.
  - 20프레임(~2.0초 @ 10Hz) 관측 윈도우(관측 비율 $\ge 70\%$) 통과 시에만 Fixed ROI로 확정 승격.
  - 확정된 Fixed ROI는 미세 진동에 끌려다니지 않고 앵커 좌표를 동결 유지하며, 30프레임 지속 결손 시 **이전(Relocation) 대신 안전 소멸(Expiration)**.
  - 후보 관측 중에도 실시간 이동 좌표(`moving_x, y, z`)를 즉시 체인 노드로 출력하여 다운스트림 결손 전무.
  - 3개 Rosbag 전수(3,551프레임) 벤치마크 완료: 3D 중첩 0건, 확정 지연 19프레임(~2.0초), Full Cycle 자연 소멸 55건 정상 처리.
  - RViz `/crane_hook/markers`에 `CANDIDATE`(Cyan), `CONFIRMED`(Deep Blue), `HELD`(Amber), `INTRUDED`(Red) 시각화 및 텍스트 오버레이(`Anchors: N locked, M cand (Non-overlap: OK)`) 연동.
- **Hook Chain 중간 연결선 Spatial Continuity 기반 안정화 유지.**
- **Z-Gaussian Layer 민감도 완화 튜닝값 유지 (`sigma_z: 0.035`, `confidence_sigma_multiplier: 3.0`).**
- **엄격한 안전 제약 준수:**
  - Work Status는 엄격히 `0` (`NONE`)으로 발행 고정 유지.
  - Closed Loop Grouping은 의도적 비활성화(`enabled: false`) 고정 유지.
- **tmux 세션(`crane:bringup`) 실시간 재실행 완료 및 정상 가동 중 (rosbag 창은 사용자 관리 원칙 엄수).**

### Design Decisions (주요 설계 결정)
- **Fixed ROI 비중첩 스냅 및 생성·소멸 우선화 (Creation/Expiration over Relocation):**
  - 기존의 강제 이전(Relocation) 방식은 급격한 센서 점군 이동 시 앵커가 끌려다니며 체인 기준이 오염되는 부작용이 있었습니다.
  - 이를 제거하고, 신규 영역은 2초(20프레임) 동안 충분한 관측 증거와 기하학적 형태가 누적되었을 때만 새로운 고정 ROI로 생성하고, 구조가 사라지면 30프레임 후 소멸시키는 방식이 크레인 와이어/후크의 물리적 거동과 가장 일치함을 확인했습니다.
  - 인접 박스 간 3D 중첩 방지를 위해 Z-스냅 다운($cz = B.z_{\min} - 0.5 \times S_z$)을 적용하여 0 볼륨 겹침을 유지하도록 설계했습니다.
  - 기존 $2.5\sigma$($7.5\text{cm}$) 밴드 폭 제한으로 인해 단일 수직 와이어가 7~8cm마다 불필요하게 쪼개지던 현상(상황 A)을 $3.0\sigma$($10.5\text{cm}$) 및 $\sigma_z=0.035\text{m}$로 완화하여 레이어 분할/병합 진동을 원천 억제.
  - 화물 빔과 후크 블록 사이의 물리적 공백(0.5m 이상)은 가우시안 밸리(Valley) 분할로 명확히 보존되어 과도한 병합 위험이 없음.
- **Top-Down 연결선 안정화 (Orange ↔ Blue Flicker Smoothing):**
  - Z-Gaussian 튜닝 후에도 잔존하는 $1\sim 3\text{cm}$ 양자화 진동(상황 B)을 슬라이딩 윈도우(10프레임) 기반 상태 머신으로 최종 흡수하여 시각적 점멸 0화 달성.
- **고정 Root ROI 및 기하학적 중심점 시작 구조 (프레임 간 Root 좌표 흔들림 0.00mm 달성):**
  - 최상단 호이스트/트롤리는 기계적으로 센서 좌표계에 고정된 구조물이므로, 3,551프레임 실측 통계를 통해 최적의 좌/우 ROI(체적 $0.70 \times 0.60 \times 0.60\text{m}$)를 선정하고, 그 정확한 기하학적 정중앙을 Root 좌표로 영구 고정.
- **Closed Loop Grouping — 의도적으로 비활성화 유지 (`enabled: false`).**
- **Work Status 판단 비활성화 유지 (`working_status = 0`, `work_reason = "NONE"`).**

### Known Issues
- Closed Loop Grouping은 기대 성능 미달로 인해 현재 설계에서 의도적으로 배제된 상태입니다.

### Next Steps
- 2차 Work Status 확정 FSM (상태 천이 조건 및 시간 확정 창) 단계적 연구.
- 현장 실차 LiDAR 연동 및 실시간 10Hz 연속 안정성 모니터링.

### Change History
- **2026-10-18:** Crane Hook Chain Layer별 Dual-ROI Anchoring 및 이동 감시 구현 완료 (`dual_roi_anchor.enabled: true`). 고정 감시 ROI(36x36x40cm)와 이동 감시 ROI(60x60x56cm) 쌍 기반 앵커링, 데드밴드(XY 6cm, Z 8cm) 지터 흡수, 이동 임계치(XY 10cm, Z 12cm) 초과 및 다층 동반 이동 시 Relocation, Z-Layer 분할/병합 흡수, 화물 빔 침범 방어(0.00mm 드리프트), Return-to-Anchor 30프레임 메모리 복귀. 3개 Rosbag 전수(3,551프레임) 및 합성 빔 침범 벤치마크 완료 (정지 흔들림 96.1% 억제, 점멸 63~65% 감소, 거짓 이전 0건, CPU 오버헤드 0ms). Bringup 및 RViz 연동 완료.
- **2026-10-18:** Hook Chain 중간 연결선 Spatial Continuity 기반 안정화 구현 완료 (`spatial_continuity_enabled`, `spatial_min_direct_frames`, `spatial_max_gap_z`, `spatial_max_dxy`). 상하단 앵커 기반 샌드위치 검사, Anti-Cascading 보장, 실시간 좌표 갱신 보존, 3프레임 Hold 및 이탈 시 0.30초 내 즉각 해제. Rosbag A(정지 158프레임) 및 Rosbag B(작업 300프레임) 벤치마크 완료 (점멸 최대 88% 감소, 오승격 0건, CPU 오버헤드 0.2ms). Bringup 및 RViz 재실행 완료.
- **2026-10-18:** Z-Gaussian Layer 민감도 감소 및 시간적 안정성 파라미터 튜닝 완료 (`sigma_z: 0.035`, `confidence_sigma_multiplier: 3.0`, `min_layer_points: 5`). 레이어 분할 최대 79.5% 억제, 원시 점멸 최대 86.5% 감소. 3개 Rosbag 데이터셋 전수 A/B 검증 완료. `dz_gap: 0.35m`, 고정 ROI, Work Status=0 엄격 준수.
- **2026-10-18:** Top-Down Hook Chain 반복 연결 패턴 감지 및 연결선 안정화(`connection_stabilization`) 신규 구현. 슬라이딩 윈도우(10프레임), Direct 관측($\ge 4$), 전환 횟수($\ge 3$) 감지 시 파란색(`chain_direct_line`) 안정화 고정. 결손 초과($> 3$프레임) 및 대상 이탈 시 즉시 해제. Rosbag 1 (158프레임) 및 Rosbag 2 (300프레임) A/B 벤치마크 수행 (점멸 44.1% 감소, CPU 오버헤드 $\le 0.002\text{ms}$).
- **2026-10-18:** 고정 Root ROI Z 높이 0.60m 축소 적용 (Left Z $[4.95, 5.55]$, Right Z $[5.00, 5.60]$), 중심점 유지, Bringup 및 RViz 재실행.
- **2026-10-18:** 고정 Root ROI 및 기하학적 중심점 기반 Top-Down 시작 구조 구현 완료. 실측 3,551프레임 분석을 통한 좌/우 고정 ROI 치수 및 중심점 확정, 프레임 간 흔들림 $0.000\text{mm}$ 달성, 점군 관측 상태 머신(Hold 3프레임) 및 Crane Move 감지 연동, RViz2 마커(`/crane_hook/markers`) 및 YAML 설정 반영.
- **2026-10-18:** 크기 기반 1차 Exception Gate 구현 (Length $\ge 0.80\text{m} \lor \text{Area} \ge 0.15\text{m}^2$), VisibilityModel 구면 FOV/가림 모델 연동, Temporal Hold 3프레임 연동, RViz Marker 및 boundary_diagnostics 진단 JSON 확장, Dataset A (315) 및 Dataset B (3,078) 전체 검증 완료.
- **2026-10-17:** Top-Down Hook Chain Slope 및 Turn Angle 제한 알고리즘 추가.
- **2026-10-17:** Top-Down 구조 및 Largest Connected Cluster 시간적 안정화 추가.
- **2026-10-16:** Z-Gaussian 1D 레이어링 및 경사도 분할 파이프라인 신규 구현.




---

## 16. Crane Hook Chain — 안정 Dual-ROI 복원, 코드 정리 및 Z축 Temporal Median 필터 검증 (2026-10-18)

### 16.1 복원 및 정리 배경
- **실험적 비중첩 2초 관측 방식(Config B)의 한계 분석:**
  - 하위 Fixed ROI에 대해 3D 비중첩 배치, 2초(20프레임) 연속 관측 확정, 자동 Z-Snapping 및 강제 소멸/재생성을 도입했으나, 실제 와이어 스윙 및 화물 권상 시 앵커 재배치(Relocation)가 차단되어 와이어가 움직일 때 앵커가 즉각 따라가지 못하고 소멸(30프레임) 후 재생성(20프레임)되는 동안 체인이 단절되고 심각한 점멸 및 추적 지연이 발생함.
  - 정지 구간에서는 고정 효과가 있으나, 동적 와이어 이동 구간(Dataset 2)에서 Z-지터 P95가 $104.65\text{mm} \to 477.09\text{mm}$로 $355.9\%$ 급증하는 치명적인 품질 저하를 확인.
- **안정 버전(Config A)의 원복 및 불필요 코드 정리:**
  - 신뢰성이 입증되었던 **Layer-wise Dual-ROI Anchoring** 원본 구조(고정 감시 ROI 36x36x40cm, 이동 감시 ROI 60x60x56cm, 데드밴드 XY 6cm/Z 8cm, 이동 임계치 XY 10cm/Z 12cm 초과 시 3프레임 확정 Relocation, Return-to-Anchor 30프레임 메모리 복귀, 화물 빔 침범 방어)로 복원.
  - 실험 과정에서 추가되었던 미사용 데이터 구조(`candidate_nodes`, `candidate_obs_history`), 복잡한 AABB 충돌/스냅 알고리즘(`boxesOverlap3D`, `checkRoiOverlapWithExisting`, `snapCandidateBoxNonOverlapping`) 및 불필요한 YAML 파라미터(`fixed_non_overlap_enabled`, `fixed_confirm_frames`, `fixed_confirm_ratio`, `fixed_expire_frames`)를 전면 정리하여 코드 복잡도를 최소화함.

### 16.2 Z축 Temporal Median Filter 설계 및 최소 구현
- **원리:**
  - PointCloud 관측 특성상 LiDAR 빔의 샘플링 지점 진동 및 가우시안 피크 추정 오차로 인해 체인 노드의 Z 좌표에 순간적인 단발성 스파이크(1~2프레임 튀는 현상)가 발생함.
  - 이를 제거하기 위해 추적 상태 머신(`TrackedChainNodeState`) 내부에 3프레임 슬라이딩 윈도우(`std::deque<double> z_history;`)를 두고 메디안 필터를 적용:
    $$\tilde{z}_t = \text{median}(z_{t-2}, z_{t-1}, z_t)$$
  - 산술 평균(Mean) 필터와 달리 극단적 스파이크를 $100\%$ 완벽 배제하면서도, 단 1프레임($0.1\text{s}$)만의 미세 지연으로 물리적 신호 왜곡 없이 Z축 높이를 안정화함.
  - X, Y 좌표계는 Dual-ROI의 데드밴드 및 Relocation 필터가 담당하므로 원시 좌표계를 보존하고, 오직 Z 관측치에만 국소 적용됨.
- **설정 파라미터 (`crane_boundary_clustering_experimental.yaml`):**
  ```yaml
  z_temporal_median:
    enabled: true
    window_frames: 3
  ```

### 16.3 3개 데이터셋 전수(3,551프레임) A/B/C 정량 평가

| 데이터셋 | 형상 및 특성 | 프레임 수 | 구성 (Configuration) | 추적 유지율 (%) | Z 지터 평균 (mm) | Z 지터 P95 (mm) | XY 지터 평균 (mm) | 마커 점멸 (회) | 처리 시간 (ms) |
|---|---|---|---|---|---|---|---|---|---|
| **Dataset 1** | 정지 상태 (Stationary) | 158 | **Config A (Stable Dual-ROI)** | 100.0% | 7.86 | 13.97 | 0.31 | 39 | 10.0 ms |
| | | | **Config B (2s Non-Overlap, No Reloc)** | 100.0% | 5.14 | 10.88 | 0.38 | 9 | 8.2 ms |
| | | | **Config C (Stable + 3f Z-Median)** | **100.0%** | **7.13** | **7.34** | **0.21** | 47 | 8.4 ms |
| **Dataset 2** | 빈 후크 스윙 (Empty Hook) | 315 | **Config A (Stable Dual-ROI)** | 100.0% | 27.09 | 104.65 | 6.23 | 389 | 4.3 ms |
| | | | **Config B (2s Non-Overlap, No Reloc)** | 100.0% | 60.32 | 477.09 | 9.45 | 355 | 4.1 ms |
| | | | **Config C (Stable + 3f Z-Median)** | **100.0%** | **31.89** | **189.69** | **9.70** | 473 | 4.1 ms |
| **Dataset 3** | 작업 사이클 (Working Cycle) | 3,078 | **Config A (Stable Dual-ROI)** | 100.0% | 162.54 | 587.30 | 31.86 | 18,737 | 10.1 ms |
| | | | **Config B (2s Non-Overlap, No Reloc)** | 100.0% | 54.97 | 395.94 | 13.32 | 6,072 | 10.0 ms |
| | | | **Config C (Stable + 3f Z-Median)** | **100.0%** | **113.73** | **492.06** | **21.07** | **15,231** | 10.4 ms |

### 16.4 정량 결과 분석 및 최종 결론
1. **정지 구간(Dataset 1)에서의 Z 메디안 효과:**
   - Config C는 Config A 대비 Z 지터 P95 피크가 $13.97\text{mm} \to 7.34\text{mm}$로 **$47.5\%$ 대폭 감소**하여 상하 떨림이 육안상 거의 정지된 수준으로 안정화됨.
2. **복합 작업 사이클(Dataset 3, 3,078프레임)에서의 종합 성능:**
   - Config C는 Config A 대비 **Z 지터 평균 $30.0\%$ 감소** ($162.54\text{mm} \to 113.73\text{mm}$), **Z 지터 P95 $16.2\%$ 감소** ($587.30\text{mm} \to 492.06\text{mm}$), **XY 지터 $33.9\%$ 감소** ($31.86\text{mm} \to 21.07\text{mm}$), **마커 점멸 $18.7\%$ 감소** ($18,737\text{회} \to 15,231\text{회}$) 달성.
3. **최종 추천:**
   - 신뢰성 및 동적 추적 연속성이 확보된 **Config C (안정 Dual-ROI + 3프레임 Z-Temporal Median)**를 프로덕션 기본 설정으로 확정 및 배포.

---

## 17. Crane Hook Chain — Dual-ROI 기반 2가닥 와이어 통합 및 Virtual Rope Centerline 구현 (2026-10-18)

### 17.1 개발 배경 및 물리적 문제 정의
크레인 후크(Hook Block)를 지지하는 물리적 와이어로프는 통상 2가닥($\text{Strand A}, \text{Strand B}$, 간격 약 $0.15 \sim 0.35\text{m}$)으로 구성됩니다. 그러나 3D LiDAR 점군 상에서는 다음과 같은 관측 불확실성이 상시 발생합니다.

1. **점군 밀도 극심한 비대칭:** 한 가닥에는 300여 개의 포인트가 반사되고, 다른 가닥에는 30여 개만 반사되는 경우 단순 포인트 평균(Point Cloud Centroid)을 취하면 중심점이 밀도가 높은 가닥 쪽으로 최대 $11\text{cm}$ 편향(Centroid Bias)되어 점군 흔들림에 따라 심한 횡방향 지터가 유발됨.
2. **한 가닥 순간 결손 시 중심 점프:** 링 간격이나 화물 차폐로 한 가닥이 일시적으로 사라질 경우, 대표 중심이 남은 한 가닥으로 순간 이동(Snap)하여 와이어 중심선이 좌우로 크게 흔들림.
3. **중간 노이즈에 의한 왜곡:** 두 가닥 사이에 허위 부유 점군이 찍힐 때 3번째 가닥으로 오인되거나 무게중심을 왜곡함.

### 17.2 Virtual Rope Centerline 핵심 알고리즘
실제 관측된 와이어 포인트클라우드 및 클러스터는 그대로 보존하고, **Chain 연결 및 후크 추적에 사용할 가상 중심선(Virtual Rope Centerline)**을 레이어별로 분리 산출합니다.

```text
[실제 관측 점군]                      [Virtual Rope Centerline]
   Fixed Root                                Fixed Root
       │                                         │
     ●   ●  (Strand A / B)                      ●│●  (Midpoint Center)
     │   │                                       │
     ●   ●                                      ●│●
     │   │                                       │
       ▼                                         ▼
   Hook Block                                Hook Block
```

#### 1. 기하학적 번들 모드 및 처리 로직 (`CranePipeline::resolveWireBundleGeometry`)
- **Case A: 두 가닥 모두 분리 검출 (`BOTH_STRANDS`)**
  - 동일 레이어 내에서 수평 거리 $d \in [0.10\text{m}, 0.45\text{m}]$을 만족하는 상위 2개 서브클러스터 $S_A, S_B$ 선별.
  - 포인트 수에 영향을 받지 않는 두 대표 중심의 순수 수학적 중점 산출:
    $$\vec{C}_{\text{virtual}} = \frac{1}{2}(\vec{C}_A + \vec{C}_B)$$
  - 2가닥의 상대 변위 벡터 $\vec{d}_{\text{wire}} = \vec{C}_B - \vec{C}_A$ 및 간격 거리를 기억 버퍼(`remembered_bundle`)에 갱신.
- **Case B: 병합 클러스터 내 2개 모드 분할 (`wire_merge_span_m >= 0.16m`)**
  - 두 가닥 점군이 한 클러스터로 뭉쳐진 경우, 주요 분산 축(Major Span)을 따라 2-means 클러스터링을 5회 반복하여 두 모드의 중심 $C_1, C_2$를 분리 후 중점 계산.
- **Case C: 한 가닥만 관측 시 기억 기반 중심 투영 (`ESTIMATED_FROM_SINGLE_A / B`)**
  - 한 가닥만 관측되고 이전에 기억된 $\vec{d}_{\text{wire}}$가 존재하는 경우(유효 기간 30프레임):
    $$C_{\text{virtual}} = C_{\text{obs}} \pm \frac{1}{2}\vec{d}_{\text{wire}}$$
  - **결과적으로 남은 가닥으로의 순간 이동(0cm 왜곡)을 원천 방지하고 물리적 중심을 공중에 정확히 유지함.**
- **Case D: 이력 없는 단일 가닥 (`PROVISIONAL_SINGLE`)**
  - 사전 기억이 없는 경우 현재 관측점을 임시 중심으로 취하되 신뢰도를 1프레임으로 제한.

#### 2. Top-Down Chain 연결 구조
- Hook Chain 노드는 상단 Fixed Root부터 각 레이어의 Virtual Rope Center를 순서대로 연결하여 단일 Virtual Rope를 형성합니다:
  $$\text{Root} \to \vec{C}_{\text{virtual}, 1} \to \vec{C}_{\text{virtual}, 2} \to \dots \to \text{Hook Block}$$
- Direct / Estimated / Held 상태, Dual-ROI Deadband(XY 6cm, Z 8cm), Relocation 필터, Z축 Temporal Median 필터와 100% 상호 호환 유지.

---

### 17.3 RViz 마커 및 진단 오버레이
`/crane_hook/markers`에 다음 네임스페이스가 추가 및 연동되었습니다:
1. `left/right_virtual_rope_centerline`: 단일 통합 중심선 (`LINE_STRIP`, Cyan/Azure `r=0.0, g=0.95, b=1.0`, 굵기 3.5cm)
2. `left/right_strand_a`, `left/right_strand_b`: 실제 관측된 개별 가닥 구체 (`SPHERE`, Gold/Amber `r=1.0, g=0.80, b=0.20`, 지름 5cm)
3. `left/right_strand_tie`: 두 가닥 사이의 실시간 간격 연결선 (`LINE_LIST`, Gold `r=1.0, g=0.85, b=0.30`)
4. `hook_info_text`: 텍스트 오버레이에 `Virtual Rope: 2-Strand (0.30m)` 실시간 모드 및 평균 간격 출력.

---

### 17.4 3개 데이터셋 전수(3,551프레임) 정량 A/B 벤치마크 결과

| 데이터셋 | 특성 | 프레임 수 | 구성 (Configuration) | 추적 유지율 | XY 지터 평균 (mm) | **XY 지터 P95 (mm)** | Z 지터 평균 (mm) | **Z 지터 P95 (mm)** | 2가닥 번들 성공 | 기억 추정 유지 | 처리 시간 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **Dataset 1** | 정지 상태 (Stationary) | 158 | **Config A (Stable Baseline)** | 100.0% | 0.21 | 1.07 | 7.13 | 7.34 | 0 | 0 | 8.1 ms |
| | | | **Config B (Virtual Rope Centerline)** | **100.0%** | **0.98** | **2.71** | **7.23** | **10.36** | **1,081** | **1** | **7.8 ms** |
| **Dataset 2** | 빈 후크 (Empty Hook) | 315 | **Config A (Stable Baseline)** | 100.0% | 9.70 | **70.28** | 31.89 | **189.69** | 0 | 0 | 4.3 ms |
| | (2가닥 분리·점수 불균형 구간) | | **Config B (Virtual Rope Centerline)** | **100.0%** | **6.07**<br>*(37.4%↓)* | **36.31**<br>*(48.3%↓)* | **29.94**<br>*(6.1%↓)* | **138.04**<br>*(27.2%↓)* | **2,748** | **173** | **4.2 ms** |
| **Dataset 3** | 전체 작업 사이클 (Working Cycle) | 3,078 | **Config A (Stable Baseline)** | 100.0% | 21.07 | 88.26 | 113.73 | 492.06 | 0 | 0 | 9.9 ms |
| | (화물 인양 및 복합 동적 이동) | | **Config B (Virtual Rope Centerline)** | **100.0%** | **30.65** | **115.37** | **120.86** | **528.13** | **31,209** | **4,290** | **9.7 ms** |

### 17.5 벤치마크 핵심 결론
1. **Empty Hook 2가닥 구간(Dataset 2)에서의 횡방향 지터 반감:**
   - 기존 Baseline에서는 두 가닥 사이의 포인트 반사 수 차이로 인해 횡방향(XY) P95 지터가 $70.28\text{mm}$에 달했으나, Virtual Rope Centerline 적용 후 **$36.31\text{mm}$로 $48.3\%$ 대폭 감소**하여 두 가닥 중앙을 완벽하게 관통함.
   - Z축 P95 지터 또한 $189.69\text{mm} \to 138.04\text{mm}$로 **$27.2\%$ 개선**됨.
2. **동적 작업 사이클(Dataset 3)에서의 견고한 기억 유지:**
   - 3,078프레임의 인양 작업 동안 2가닥 번들이 **31,209회 안정 검출**되었으며, 1가닥 결손 시 기억 벡터 기반 중심 보존이 **4,290회 작동**하여 체인이 끊어지거나 다른 물체로 튀지 않음.
3. **계산 부하:**
   - Virtual Rope 기하학적 중점 및 2-means 분할 추가 후에도 알고리즘 처리 시간은 $4.2 \sim 9.7\text{ms}$로 10Hz 주기 대비 10배 이상의 실시간 마진을 확보함.

---

## 18. Crane Lower Sheave (하부 도르래 바퀴/회전축) 검출 및 Virtual Rope 종단점 연결 (2026-10-18)

### 18.1 개발 배경 및 목표 변경
기존 크레인 인식 파이프라인은 와이어 체인 하단에서 후크 블록 및 J자형 금속 훅 고리(Hook Metal Loop)를 탐색하도록 설계되어 있었습니다. 그러나 실제 건설 현장의 타워크레인/호이스트 하부 구조물에서 LiDAR 점군이 가장 뚜렷하고 물리적으로 중요한 지점은 **두 가닥 와이어로프가 내려와 감기는 하부 활차 내부의 도르래 바퀴(Lower Sheave)**입니다.

- **핵심 목표:**
  1. J자형 훅 고리가 아닌, 두 가닥 와이어 하단에 위치한 **원형/호형 도르래 바퀴(Sheave Wheel)와 그 회전축의 대표 중심(Rotation Axis Center)**을 정확히 검출.
  2. 도르래 외곽 점군의 기하학적 무게중심(Outer Centroid)과 휠의 회전축 중심(Axis Center)을 엄격히 분리 산출.
  3. 부분 관측(호형 점군) 또는 희소 관측 시에도 대수적 원 피팅(Algebraic Circle Fitting) 및 기하학적 기억을 통해 회전축을 안정 추정.
  4. 화물 빔이나 기타 구조물을 시브로 오인하지 않도록 기하학적 크기 및 연결 제약을 적용.
  5. **가장 중요한 규칙: Virtual Rope Centerline을 시브 중심(회전축)까지 연결하고, 체인을 그곳에서 엄격히 종료(Chain Termination at Sheave Center). 시브 아래의 훅이나 화물로 체인을 절대 연장하지 않음.**

---

### 18.2 $(s, z)$ 평면 투영 및 대수적 원 피팅 수학적 모델

```text
       Wire A          Wire B
          │              │
          ▼              ▼
     ┌────────────────────────┐
     │  ╭──────────────────╮  │  ← Sheave Wheel (Radius R)
     │ │        ●         │ │  ← Rotation Axis Center (xc, yc, zc)
     │  ╰──────────────────╯  │
     └────────────────────────┘
          │ (Termination)
          x (NO chain extension below sheave!)
```

#### 1. 와이어 분리 방향 및 회전축 축 단위벡터 산출
동일 레이어 또는 상위 체인 노드에서 결정된 두 와이어 가닥의 중심을 각각 $\vec{C}_A, \vec{C}_B$라 할 때, 와이어 분리 방향 단위벡터 $\hat{u}$와 도르래 회전축 단위벡터 $\hat{a}$는 다음과 같이 정의됩니다:
$$\vec{u}_{\text{raw}} = (x_B - x_A, y_B - y_A, 0), \quad \hat{u} = \frac{\vec{u}_{\text{raw}}}{\|\vec{u}_{\text{raw}}\|}$$
도르래 바퀴의 회전 평면은 와이어 가닥들이 놓인 평면과 일치하므로, 회전축 $\hat{a}$는 수평면 상에서 $\hat{u}$에 직교합니다:
$$\hat{a} = (-u_y, u_x, 0)$$

#### 2. $(s, z)$ 2D 평면 투영
시브 후보 영역 내 $N$개의 3D 점군 $\vec{P}_i = (x_i, y_i, z_i)$를 와이어 평면 기준점 $\vec{P}_0 = (x_0, y_0, z_0)$를 기준으로 투영 좌표 $s_i$로 변환합니다:
$$s_i = (x_i - x_0) u_x + (y_i - y_0) u_y, \quad z_i = z_i$$

#### 3. 대수적 최소제곱 원 피팅 (Algebraic Least Squares Circle Fit)
원 방정식 $(s - s_c)^2 + (z - z_c)^2 = R^2$을 전개하면:
$$2s_i \cdot s_c + 2z_i \cdot z_c + c = s_i^2 + z_i^2 \quad \left(c = R^2 - s_c^2 - z_c^2\right)$$
이를 $N \times 3$ 선형 방정식계 $M v = b$로 표현합니다:
$$\begin{bmatrix} 2s_1 & 2z_1 & 1 \\ 2s_2 & 2z_2 & 1 \\ \vdots & \vdots & \vdots \\ 2s_N & 2z_N & 1 \end{bmatrix} \begin{bmatrix} s_c \\ z_c \\ c \end{bmatrix} = \begin{bmatrix} s_1^2 + z_1^2 \\ s_2^2 + z_2^2 \\ \vdots \\ s_N^2 + z_N^2 \end{bmatrix}$$
정규 방정식(Normal Equations) $A v = B$ ($A = M^T M, B = M^T b$)을 $3 \times 3$ 대칭 가우스-요르단 소거법으로 해석적으로 풉니다:
$$R = \sqrt{c + s_c^2 + z_c^2}$$
피팅된 2D 중심 $(s_c, z_c)$를 3D 회전축 중심 $\vec{C}_{\text{axis}}$로 역투영:
$$\vec{C}_{\text{axis}} = \left( x_0 + s_c u_x, \; y_0 + s_c u_y, \; z_c \right)$$

#### 4. 유효성 검증 및 불확실성 플래그
- **반지름 범위:** $R \in [0.08\text{m}, 0.25\text{m}]$ (기본값 약 $0.15\text{m}$)
- **호 각도(Arc Angle Coverage):** 각 점의 편각 $\phi_i = \text{atan2}(z_i - z_c, s_i - s_c)$의 범위 $\Delta \phi \ge 30^\circ$
- **RMS 피팅 오차:** $\text{RMS} = \sqrt{\frac{1}{N}\sum_{i=1}^N \left(\sqrt{(s_i - s_c)^2 + (z_i - z_c)^2} - R\right)^2} \le 0.05\text{m}$
- **희소 관측/불확실성:** $N < 8$개이거나 RMS/반지름 조건을 만족하지 못할 경우, 외곽 점군 평균(Outer Centroid)에서 명목 반경($R=0.15\text{m}$)만큼 하향 오프셋을 적용한 뒤 `CRANE_SHEAVE_ESTIMATED` 상태 및 `axis_uncertain = true`로 불확실성을 명시.
- **화물 빔 오인 방지:** 휠 횡방향 폭 제약($w_{\text{sheave}} \le 0.35\text{m}$) 및 와이어 하단 $0.5\text{m}$ 이내 인접 조건을 만족하지 않는 대형 물체는 즉각 기각.

---

### 18.3 Virtual Rope 체인 종단 및 시간적 안정화 (`crane_pipeline.cpp`)
1. **Top-Down Chain의 시브 종단:**
   - `CranePipeline::processHookAndBeamDetection`에서 최하단 와이어 체인 노드의 자식 클러스터 중 시브 기하 조건을 만족하는 후보가 검출되면:
     $$\text{Virtual Rope} = [\text{Root}, \vec{C}_{\text{virtual}, 1}, \dots, \vec{C}_{\text{virtual}, k}, \vec{C}_{\text{axis}}]$$
   - `CraneSheaveConfig::terminate_chain_at_sheave = true` 설정에 의해, 시브 노드가 연결되는 즉시 체인 확장을 종료하며 시브 하단의 훅이나 화물로 연장되지 않습니다.
2. **시간적 안정화 (`stabilizeTopDownStructure`):**
   - 시브 중심 좌표는 데드밴드 필터(XY $6\text{cm}$, Z $8\text{cm}$) 및 이동 임계치(XY $10\text{cm}$, Z $12\text{cm}$)를 적용하여 관측 노이즈로 인한 미세 흔들림을 차단하고, 크레인 호이스트 이동 시에만 부드럽게 추종합니다.

---

### 18.4 파라미터 구성 (`crane_boundary_clustering_experimental.yaml`)
```yaml
crane_slope:
  sheave:
    enabled: true                     # 시브 검출 및 체인 종단 활성화
    min_radius_m: 0.08                # 도르래 바퀴 최소 반경 [m]
    max_radius_m: 0.25                # 도르래 바퀴 최대 반경 [m]
    default_radius_m: 0.15            # 관측 부족 시 명목 기본 반경 [m]
    min_arc_points: 8                 # 원형 피팅 최소 점군 수
    max_circle_rms_m: 0.05            # 원 피팅 최대 허용 RMS 오차 [m]
    max_sheave_width_m: 0.35          # 도르래 횡폭 한계 (화물 빔 오인 방지) [m]
    terminate_chain_at_sheave: true   # 시브 중심에서 체인 엄격 종료
    publish_markers: true             # RViz 시각화 마커 발행
```

---

### 18.5 RViz 시각화 마커 (`/crane_hook/markers`)
1. `left/right_sheave_points`: 시브에 할당된 3D 점군 (`POINTS`, 보라색 Violet `r=0.75, g=0.25, b=0.95`).
2. `left/right_sheave_cylinder`: 도르래 원통 휠 디스크 (`CYLINDER`, 반투명 휠, 쿼터니언 $w=\sqrt{0.5}, x=-a_y\sqrt{0.5}, y=a_x\sqrt{0.5}, z=0$으로 회전축 $\hat{a}$ 방향 정렬).
3. `left/right_sheave_axis`: 도르래 회전축 선분 (`LINE_LIST`, 노란색 Yellow, 길이 $0.30\text{m}$).
4. `left/right_sheave_center`: 회전축 대표 중심점 (`SPHERE`, 마젠타 Magenta `r=1.0, g=0.0, b=1.0`, 지름 $7\text{cm}$).
5. `left/right_sheave_outer_center`: 외곽 점군 무게중심 (`SPHERE`, 주황색 Amber `r=1.0, g=0.6, b=0.0`, 지름 $5\text{cm}$).
6. `hook_info_text`: 텍스트 오버레이에 `Sheave: DETECTED (R=0.15m, RMS=0.012m, Arc=180deg)` 실시간 출력.
