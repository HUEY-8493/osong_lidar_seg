# Walkthrough - Roadside LiDAR Perception Improvements & Verification

본 문서에서는 `cx128s2_bev_perception_node.cpp` 코드에 반영된 개선 패치 내역, 파라미터 동적 변경 기능 적용 상태, 알고리즘적 한계, 그리고 rosbag을 이용한 비교 성능 검증 방법을 안내합니다.

---

## 1. 구현된 주요 변경 사항

### ① 신규 전경 분류 모드 `"gdg_point_pdf"` 추가
* **수정 위치:** `classifyForeground` 함수 내부 ([cx128s2_bev_perception_node.cpp:L1380-L1432](file:///home/sjjin9384/construct_ws/src/cx128s2_bev_perception/src/cx128s2_bev_perception_node.cpp#L1380-L1432))
* **기능 설명:** 
  * 본 노드는 논문 2(GDG)의 전체 구조를 그대로 복제한 것이 아니라, **GDG의 핵심 아이디어인 '3D Voxel 차이 확인 후 개별 포인트별 Gaussian PDF 높이 판정 검사' 개념을 차용하여 기존 ROI-BEV 그리드 위에 일부 결합**한 구현 방식입니다.
  * Voxel 개수 차이가 큰 유효 셀에 한해서 **개별 포인트($p_i$) 단위로 Gaussian 확률 밀도 함수(PDF)**를 직접 계산합니다.
  * 계산된 PDF 값이 아래의 임계치 조건 이하인 경우만 전경(`foreground_idx`)으로 분류하여, 차량 주변의 아스팔트나 연석 지면 포인트가 뭉뚱그려 전경에 잡히는 오버세그멘테이션(Over-segmentation) 현상을 억제합니다.
    $$P(z_i) < \text{foreground\_pdf\_density\_threshold\_} \times bg.max\_density$$

### ② 신규 ROS2 파라미터 및 런타임 동적 파라미터 조정 콜백 (`onSetParameters`) 구현
* **수정 위치:** 생성자(L396)에서의 콜백 등록 및 `onSetParameters` 구현부 (L2505-2555)
* **기능 설명:** 
  * 기존에는 `ros2 param set`을 수행하더라도 내부 C++ 멤버 변수가 실시간으로 갱신되지 않았으나, **신규 파라미터 콜백을 통해 노드 재기동 없이 즉시 변경이 반영**됩니다.
  * **보안/안전 검증 규칙:**
    * `foreground_classifier_mode`: `legacy_threshold`, `statistical_score`, `gdg_point_pdf` 값만 유효하게 허용하며 그 외의 값은 반려(Reject) 처리합니다.
    * 수치형 threshold/weight 류: 음수 입력을 방지합니다.
    * `foreground_min_persistence_frames`: 최소 `1` 이상이어야 합니다.
    * `ror_radius`: 최소 `0.05` 이상이어야 합니다.
  * 지원 파라미터: `foreground_classifier_mode`, `foreground_pdf_density_threshold`, `foreground_voxel_diff_threshold`, `foreground_min_persistence_frames`, `enable_radius_outlier_removal`, `ror_radius`, `ror_min_neighbors`.
  * Lifetime이 보존되도록 `param_callback_handle_` 멤버 변수에 바인딩을 관리합니다.

### ③ 셀 단위 지속성(Persistence) 우회로 이동 객체 누락 가능성 완화
* **수정 위치:** `classifyForeground` 내부
* **기능 설명:** 
  * `gdg_point_pdf` 모드에서는 point-level foreground 분류 결과를 cell-level persistence 필터에 다시 통과시키지 않도록 하여, 이동 객체가 셀을 이동하면서 foreground 확정 전에 누락될 가능성을 완화합니다.

### ④ `bg.max_density` 안전성 Fallback 강화
* **수정 위치:** `classifyForeground` 분기 내부
* **기능 설명:** 로드된 배경 맵의 캐싱 데이터 손상으로 `bg.max_density`가 0.0 이하이거나 누락된 경우, 런타임에 `normalPdf`를 실시간 재계산하여 Fallback 임계값을 구하도록 예외 처리를 추가했습니다.
  ```cpp
  const double max_density = bg.max_density > 1e-9 ? bg.max_density : normalPdf(bg.mean_z, bg.mean_z, bg_std);
  ```

---

## 2. 알고리즘적 한계 (Single Gaussian의 제약)

본 검출 방식은 각 BEV 격자 내의 정적 배경 높이(z)를 단일 Gaussian 모델로 추정합니다. 따라서 평평한 도로면에서는 매우 정밀하게 동작하나, 다음과 같은 수직 기하 구조를 지닌 영역에서는 한계가 존재합니다.
* **Curb(연석), 방음벽, 경계석, 신호등 기둥 인접 격자:** 격자 내부에 도로 표면과 수직 구조물이 공존하여 높이 표준편차(`std_z`)가 크게 잡힐 수 있으며, 이로 인해 지면 포인트의 PDF가 높게 판정되어 전경 검출에서 누락되거나 구조물 주변의 전경 노이즈가 발생할 수 있습니다.
* **해결 방안:** 이와 같은 복잡한 z분포 한계를 완벽히 해결하기 위해서는 장기적으로 단일 Gaussian 대신 z-Histogram 기반 Quantile Band 기법이나 Gaussian Mixture Model(GMM)로의 확장이 추천됩니다.

---

## 3. 검증 및 구동 가이드

개발 환경 상의 터미널 샌드박스 보안 제약으로 인해 자동 빌드는 생략되었으므로, **로컬 ROS2 워크스페이스 터미널에서 아래의 단계에 따라 직접 빌드 및 검증**해 주시기 바랍니다.

### ① ROS2 워크스페이스 빌드
```bash
cd ~/construct_ws
colcon build --symlink-install --packages-select cx128s2_bev_perception
```

### ② 패키지 실행 및 파라미터 주입
새로운 `gdg_point_pdf` 모드로 노드를 실행하기 위해 Launch 파일에 파라미터를 추가하거나, 다음과 같이 런타임에 직접 실행 인자로 주입할 수 있습니다:
```bash
ros2 run cx128s2_bev_perception cx128s2_bev_perception_node --ros-args \
  -p foreground_classifier_mode:="gdg_point_pdf" \
  -p foreground_pdf_density_threshold:=0.3 \
  -p foreground_voxel_diff_threshold:=3
```

---

## 4. rosbag 비교 검증 플랜 (Comparative Verification Plan)

rosbag을 구동한 상태에서 아래 3가지 모드의 파라미터 조정을 수행하며 `/debug/perception_diagnostics_table` 토픽의 지표 변화와 RViz2 시각화 화면을 대조 평가합니다.

### 1) 비교 시험 구성 및 명령어

* **모드 A: 기존 디폴트 (지속성 3)**
  ```bash
  ros2 param set /cx128s2_bev_perception foreground_classifier_mode statistical_score
  ros2 param set /cx128s2_bev_perception foreground_min_persistence_frames 3
  ```
* **모드 B: 기존 구조 + 지속성 해제 (지속성 1)**
  ```bash
  ros2 param set /cx128s2_bev_perception foreground_classifier_mode statistical_score
  ros2 param set /cx128s2_bev_perception foreground_min_persistence_frames 1
  ```
* **모드 C: 신규 패치 모드 (Point-level Gaussian PDF)**
  ```bash
  ros2 param set /cx128s2_bev_perception foreground_classifier_mode gdg_point_pdf
  ros2 param set /cx128s2_bev_perception foreground_pdf_density_threshold 0.3
  ros2 param set /cx128s2_bev_perception foreground_voxel_diff_threshold 3
  ```

### 2) 모니터링 및 성능 비교 지표

`/debug/perception_diagnostics_table` 토픽을 서브스크라이브하여 다음 항목들을 모니터링합니다:
* **`foreground_point_count`:** 
  * 모드 C는 차량 주변 지면 포인트가 과도하게 foreground로 포함되는 경우를 줄이는 것이 목표입니다. 따라서 단순 foreground point count 감소만 보지 말고, RViz에서 차량 하단 ground 혼입 여부, cluster bbox 크기, candidate/track 안정성, false positive 증가 여부를 함께 평가해야 합니다. (unknown cell이 많거나 background 학습이 부족하면 모드 C에서 전경 포인트가 일부 늘어날 수도 있으므로 종합적 검증이 중요합니다).
* **`candidate_count` / `active_track_count`:**
  * 주행 중인 차량에 대한 트랙 생성 안정성을 비교합니다. 모드 A의 경우 이동 차량 트랙이 계속 끊기거나 소실되나, 모드 B와 모드 C는 안정적으로 주행 차량의 트랙이 유지되어야 합니다.
* **`foreground_persistence_bypassed`:**
  * 모드 C에서 지속성 필터 우회가 켜져 있는지(`1`) 확인합니다.
* **추가 GDG Diagnostics 지표:**
  * `gdg_pdf_tested_cell_count` (GDG PDF 경로로 진입한 유효 격자 수)
  * `gdg_pdf_foreground_cell_count` (전경 포인트가 1개 이상 검출된 격자 수)
  * `gdg_pdf_tested_point_count` (PDF 계산이 이루어진 총 포인트 수)
  * `gdg_pdf_foreground_point_count` (PDF 검사를 통과해 전경으로 잡힌 포인트 수)
  * `gdg_pdf_background_point_count` (PDF 검사를 통해 정적 배경으로 배제된 포인트 수)
  * `gdg_pdf_unknown_cell_point_count` (배경 정보가 없어 통전경 처리된 포인트 수)
  * `gdg_pdf_count_diff_rejected_cell_count` (Voxel 개수 차이가 작아 PDF 계산 없이 배경 기각된 셀 수)
