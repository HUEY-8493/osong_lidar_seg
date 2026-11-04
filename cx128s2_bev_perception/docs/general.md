# CX128S2 BEV Perception — General Mode 개발 문서

> **문서 버전:** 1.0.0  
> **최종 갱신일:** 2026-10-18  
> **대상 패키지:** `cx128s2_bev_perception`  
> **핵심 실행 프로파일:** `perception_mode: "default"`  
> **주요 소스 파일:**
> - [src/cx128s2_bev_perception_node.cpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/cx128s2_bev_perception_node.cpp)
> - [include/cx128s2_bev_perception/vehicle_surface_tracking.hpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/vehicle_surface_tracking.hpp)
> - [include/cx128s2_bev_perception/visibility_model.hpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/visibility_model.hpp)
> - [src/visibility_model.cpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/src/visibility_model.cpp)
> - [include/cx128s2_bev_perception/linear_kalman_filter.hpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/linear_kalman_filter.hpp)
> - [include/cx128s2_bev_perception/raw_vehicle_box_refinement.hpp](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/include/cx128s2_bev_perception/raw_vehicle_box_refinement.hpp)
> - [config/cx128s2_bev_perception.yaml](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/config/cx128s2_bev_perception.yaml)
> - [config/cx128s2_roi_policy.yaml](file:///home/sjjin/construct_ws/src/cx128s2_bev_perception/config/cx128s2_roi_policy.yaml)

---

## 1. General 개발 목적

General 모드는 도로변 지주(Pole) 또는 공사현장 게이트에 고정 설치된 CX128S2 3D 라이다로부터 진입/진출하는 차량(트럭, 건설차량 등)을 실시간으로 감지하고 추적하기 위해 설계되었습니다.

1. **정적 배경 맵 학습 및 차분 (Static Background Subtraction):**
   - 도로변의 정적 구조물(가드레일, 벽체, 연석, 바닥 지면)을 오프라인/온라인으로 사전 학습하여 고정밀 Voxel 그리드로 동결하고, 동적 이동 물체만을 전경(Foreground)으로 안정적으로 추출.
2. **BEV 연결 요소 클러스터링 (BEV Connected Component Clustering):**
   - 2D Bird's Eye View 격자에서 Z-Guard(높이 차이 임계치) 및 Gap-Bridging(가상 브릿지 연결)을 적용하여 분절 없는 차량 덩어리 형성.
3. **선형 칼만 필터 기반 차량 추적 (Linear Kalman Tracking):**
   - Constant-Velocity 등속도 운동 모델 기반 칼만 필터를 사용하여 근거리/원거리 차량의 위치 및 주행 속도 추정.
4. **차량 생애주기 관리 및 재획득 (Lifecycle & Re-acquisition):**
   - 가림(Occlusion)이나 사각지대로 일시적으로 시야에서 사라진 차량을 `LOST` 예측 단계에서 추적하고, 예측 범위를 벗어난 차량을 `DORMANT`(동결된 메모리) 상태로 수 시간 이상 기억하여 동일 차량 재등장 시 기존 고유 ID로 복원.
5. **가시성 모델 (Visibility Model):**
   - 라이다 센서 채널 광선(Ray)의 고도각/방위각 프로파일을 모델링하여 물리적으로 센서 시야에 가려진 은폐 영역과 실제 빈 공간을 명확히 구분.

---

## 2. General 처리 흐름

실제 C++ 코드 상에서 센서 콜백부터 트랙 발행까지의 함수 호출 순서는 다음과 같습니다.

```text
cx128s2_bev_perception_node.cpp: pointCloudCallback()
        │
        ▼ (Target Frame TF 변환: world)
passValidCrop()
        │
        ▼ (perception_mode_ != "crane_beam_work")
applyRoiPolicyToIndices() (다중 영역 ROI 필터링: WATCH, IGNORE, HARD_IGNORE)
        │
        ▼
accumulateBackground() / finalizeBackground() (수동 명령 시 배경 학습)
        │
        ▼
classifyForeground() (통계 점수 또는 GDG Point PDF 방식 전경/배경 분리)
        │
        ▼ (선택적)
radiusOutlierRemoval() (반경 이상치 제거 ROR)
        │
        ▼
clusterForeground() (BEV 4/8-way 연결요소 또는 Shape Graph 클러스터링)
        │
        ▼
computeClusterFeatures() (중심점, 2D Convex Hull, 최소 외접 Bounding Box 계산)
        │
        ▼ (선택적)
refineRawVehicleBoxes() (원시 3D 점군 기반 OBB 보정)
        │
        ▼
classifyObjects() (차량 규격 필터링: 길이, 폭, 높이, 점수)
        │
        ▼
updateTracks() (차량 트랙 갱신 메인 진입점)
        ├── predictKalmanForTrack() (모든 트랙 상태 사전 칼만 예측)
        ├── DORMANT 전환 검사 (LOST 예측 기한 초과 시 위치 동결)
        ├── advanceCorridorDirection() (진입 ENTRY / 진출 EXIT 방향 판정)
        ├── scoreReacquisition() (LOST/DORMANT 트랙과 관측치 매칭 점수 산출)
        ├── 헝가리안/탐욕 연관 (Candidate와 Track 간 거리/형상 매칭)
        ├── 트랙 신규 생성 (tentative_confirm_hits 카운트 시작)
        └── kf.update() (칼만 필터 위치 및 가로방향 앵커 업데이트)
        │
        ▼
updateVisibilityModel() (라이다 빔 가림 상태 갱신)
        │
        ▼
발행 (Publishers)
        ├── /detected_objects (현재 프레임 검출 배열)
        ├── /tracked_objects (활성 추적 객체 배열)
        ├── /road_vehicle_tracks (확정된 차량 트랙 배열)
        ├── /vehicle_identity_memory (DORMANT 포함 전체 차량 식별자 메모리)
        ├── /debug/object_markers (RViz 시각화 바운딩 박스)
        └── /debug/foreground_points (전경 포인트클라우드)
```

---

## 3. General 기능별 개발 현황

| 기능 | 현재 상태 | 구현 위치 | 검증 여부 | 비고 |
|---|---|---|---|---|
| **TF 및 입력 Crop** | `Implemented + Active` | `node.cpp: pointCloudCallback()` | **Verified** | 센서-월드 TF 변환, 유효 X/Y/Z 및 거리 필터링 |
| **다중 영역 ROI 정책** | `Implemented + Active` | `node.cpp: applyRoiPolicyToIndices()` | **Verified** | `cx128s2_roi_policy.yaml` 기반 WATCH/IGNORE/HARD_IGNORE |
| **정적 배경 맵 학습** | `Implemented + Active` | `node.cpp: accumulateBackground()` | **Verified** | `/learn_static_background` 트리거 시 20초간 학습 후 YAML 저장 |
| **Statistical Score 전경 분리**| `Implemented + Active` | `node.cpp: classifyForeground()` | **Verified** | Voxel 카운트, Mean Z, Max Z 가중 점수 기반 판정 |
| **GDG Point PDF 분리** | `Implemented + Experimental` | `node.cpp: classifyForeground()` | Experimental | 개별 포인트 단위 1D 가우시안 PDF 계산 모드 |
| **지면 분리 (Ground Split)** | `Implemented + Active` | `node.cpp: splitGroundObstacle()` | **Verified** | 배경 미학습 시 격자 최소 Z 기반 지면 제거 폴백 |
| **BEV 연결요소 클러스터링** | `Implemented + Active` | `node.cpp: clusterForeground()` | **Verified** | 4-way 연결, Z-guard ($0.4\text{m}$), Gap Bridging ($1\text{cell}$) |
| **Shape Graph 클러스터링** | `Implemented + Experimental` | `node.cpp: clusterShapeGraph()` | Experimental | 노드-엣지 그래프 기반 대형 차량 연결 클러스터링 |
| **원시 점군 OBB 보정** | `Implemented + Active` | `raw_vehicle_box_refinement.hpp` | **Verified** | 3D 점군 기반 차량 Bounding Box 중심 및 치수 미세 정렬 |
| **Linear Kalman Tracker** | `Implemented + Active` | `node.cpp: updateTracks()`, `linear_kalman_filter.hpp` | **Verified** | 등속도 선형 칼만 필터 ($q=1.5\,\text{m/s}^2, r=0.16\text{m}$) |
| **차량 재획득 (Re-acquisition)**| `Implemented + Active` | `vehicle_surface_tracking.hpp: scoreReacquisition` | **Verified** | 가림 후 재등장 차량의 OBB/표면 매칭 및 기존 ID 복원 |
| **DORMANT 장기 메모리** | `Implemented + Active` | `node.cpp: updateTracks()` | **Verified** | 최대 36,000초(10시간) 동안 주차/동결된 차량 ID 유지 |
| **Visibility Model** | `Implemented + Active` | `visibility_model.cpp: VisibilityModel` | **Verified** | 센서 시선(Ray) 기준 은폐(Shadow) 영역 판별 및 표시 |
| **Object Memory Bank** | `Implemented + Experimental`| `node.cpp: handleCommitShapeMemory()` | Experimental | 학습된 차량 형상 템플릿 YAML 영구 저장 및 재로딩 |

---

## 4. General Tracking 및 재획득(Re-acquisition) 상세

### 4.1 생애주기 상태 머신 (Vehicle Lifecycle)
`vehicle_surface_tracking.hpp`에 정의된 `VehicleLifecycle` 열거형에 따라 관리됩니다.

```text
       [신규 관측]
           │
           ▼
     TENTATIVE (미확정)
           │ (tentative_confirm_hits >= 28 / 31 frames)
           ▼
        ACTIVE (정상 추적)
           │
           ├─ (관측 누락 발생) ──► LOST (칼만 속도 기반 위치 예측, 최대 2.0초)
           │                         │
           │                         ├─ (유효 매칭 재관측) ──► ACTIVE 복원 (Re-acquired)
           │                         │
           │                         ▼ (예측 기한 2.0초 초과)
           │                       DORMANT (위치 동결, 메모리 보존, 최대 36,000초)
           │                         │
           │                         ├─ (정지 위치 부근 재관측) ──► ACTIVE 복원
           │                         │
           │                         ▼ (10시간 초과 시)
           └────────────────────────► 소멸 (Erase)
```

### 4.2 거리 대역별(Near/Mid/Far) 게이트 및 연관 파라미터
라이다 점군 밀도는 거리에 따라 급격히 감소하므로, 거리 대역에 따라 차등화된 임계치를 적용합니다.

- **Near Range ($< 20\text{m}$):**
  - 클러스터 최소 포인트 수: 10점
  - 최소 셀 수: 3셀
  - Gap-Bridging 허용 셀: 0셀 (근거리에서는 가상 브릿지를 금지하여 인접 물체 병합 방지)
  - 최대 Lost 허용 프레임: 15프레임
  - 연관 게이트: $d^2 \le 1.0\,\text{m}^2$
- **Mid Range ($20\text{m} \sim 35\text{m}$):**
  - 클러스터 최소 포인트 수: 6점
  - 최소 셀 수: 2셀
  - Gap-Bridging 허용 셀: 1셀 (중간 빈 셀 1칸 건너뜀 허용)
  - 최대 Lost 허용 프레임: 25프레임
  - 연관 게이트: $d^2 \le 45.0\,\text{m}^2$
- **Long Range ($35\text{m} \sim 100\text{m}$):**
  - 클러스터 최소 포인트 수: 4점
  - 최소 셀 수: 1셀
  - Gap-Bridging 허용 셀: 1셀
  - 최대 Lost 허용 프레임: 40프레임
  - 연관 게이트: $d^2 \le 70.0\,\text{m}^2$

### 4.3 재획득(Re-acquisition) 알고리즘
- **함수:** `cx128s2_bev_perception::vehicle_surface::scoreReacquisition()`
- **평가 요소:**
  1. **위치 오차:** 예측 위치 대비 관측 중심점의 유클리드 거리.
  2. **가로방향(Lateral) 오차:** 도로 진입 차선 축(Lateral Anchor) 기준 이탈도.
  3. **형상 일치도:** OBB 길이/폭의 비율 일치성 (0.8배 이상).
  4. **기억된 표면 점군 거리:** 차량이 사라지기 직전에 기억된 외곽 표면 점군(`local_surface`)과의 최근접 거리.
- **판정:**
  - 최고 점수 후보의 스코어가 기준치를 넘고, 2순위 후보와의 마진이 확보되면 `ReacquireDecision::ACCEPTED`로 판정하여 이전 트랙 번호(ID)를 그대로 부여하고 `ACTIVE`로 전환.

---

## 5. 관리 영역

### Current Status
- General 도로변 차량 감지, 정적 배경 학습, BEV 클러스터링, 선형 칼만 필터 및 재획득 파이프라인 안정 동작 중.
- 기본 설정 파일 `config/cx128s2_bev_perception.yaml`을 통해 독립적으로 구동 가능.

### Known Issues
- 안개나 강우 등 악천후 환경에서 정적 배경 맵에 포함되지 않은 아스팔트 반사 노이즈가 발생할 경우 미소 전경 클러스터가 생성될 수 있음 (ROR 필터 또는 통계 임계치 튜닝 필요).

### Next Steps
- Shape Graph 기반 대형 트레일러/트럭 연결성 향상 알고리즘 현장 실차 데이터 추가 검증.
- GPU 기반 PCL 가속 모듈 연동 검토.

### Change History
- **2026-10-14:** General 모드 파라미터 및 레코딩 인터페이스 정리.
- **2026-10-14:** `vehicle_require_entry_direction: false` 옵션 적용으로 시동 초기 주차된 차량의 즉각 감지 지원.
- **2026-10-13:** DORMANT 차량 재획득 스코어링 및 가로방향 앵커 업데이트 로직 개선.
- **2026-09-30:** Visibility Model 채널 광선 프로파일 및 마커 시각화 연동.
- **2026-09-09:** `LinearKalmanFilter` 도입 (등속도 모델 기반 트래킹 전환).

