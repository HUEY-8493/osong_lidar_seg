> 기본 실행은 이제 `../general_perception_studio`의 네이티브 ImGui Studio다. 이 문서는 보존된 Qt reference/backend 설명이다. Qt를 실행하려면 `start_general_debug_lab.sh --qt`를 사용한다. [네이티브 실행 및 기능 대응](../general_perception_studio/README.md)을 참고한다.

# General · Perception Lab

준비된 영구 데이터셋과 기존 C++ Replay 엔진을 사용하는 네이티브 Qt 데스크톱 도구. 웹 서버·QtWebEngine·브라우저를 사용하지 않는다.

```bash
/home/sjjin/construct_ws/src/cx128s2_bev_perception/test/start_general_debug_lab.sh
```

런처가 ROS 2 Jazzy와 workspace 환경을 로드한다. 전용 C++ 진단 빌드는 변경이 없으면 재사용한다. 추가 저장소는 `--dataset-root /path`, 실험/파생 결과 저장소는 `GENERAL_DEBUG_LAB_STATE`로 지정한다. 같은 state에서는 한 GUI만 실행한다.

## 화면 사용

- **Explore**: 단일 BEV와 Track Inspector. 상단에서 Dataset·Run 선택, 하단에서 프레임·상대 시간·재생 속도를 조작한다. Track 목록 또는 장면을 클릭해 선택한다.
- **Compare**: 저장 결과를 좌우로 비교한다. Before는 상단 Run, Candidate는 오른쪽 장면 헤더에서 선택한다. 프레임·줌·이동을 동기화하며 Inspector에 변경된 값만 요약한다. 원본 상세 차이는 Advanced에서 확인한다.
- **Tune**: 왼쪽 장면과 오른쪽 Parameter Lab. 검색 후 현재값과 Override를 비교한다. 상세 프레임 구간·실험 이름을 지정하고 Frame what-if 또는 Dataset replay를 실행한다. 더보기 메뉴에서 Full replay와 모든 파라미터 메타데이터를 연다.
- **Sidebar**: Dataset 메타데이터/Refresh, 저장 Experiments, Comparisons, Workspace 설정. 왼쪽 위 아이콘으로 접을 수 있고 작은 창에서는 자동으로 접힌다.

## Inspector와 장면

Inspector는 Track 상태 → 기록된 Detection/Association → 실패 Gate → KF/Geometry → Gate별 결과 순서다. 실행되지 않은 Gate는 기본적으로 묶고, 펼치거나 Details에서 `NOT_EVALUATED`를 확인한다. 기록되지 않은 최종 Association 원인은 추정하지 않는다.

장면 상단 아이콘: Layers, Fit scene, Focus selected track, Export image, 전체 화면 Focus. Layers 팝오버에 표시 레이어와 색 범례가 있다. 휠로 확대하고 드래그로 이동하며 Cluster/Track hover로 실제 기록을 확인한다. 선택한 Track만 이름과 Geometry/KF를 강조해 중첩을 줄인다. Depth는 실제 Angular/Depth 화면으로 전환한다.

F11 또는 장면의 전체 화면 아이콘으로 Focus View를 열고 Esc로 돌아온다. 하단 Events에서 기록된 이벤트를 필터하고 더블클릭하면 해당 프레임과 Track으로 이동한다.

Inspector의 더보기 또는 Advanced에서 전체 숫자 표, Raw JSON, Comparison evidence, Raw gates를 확인한다. 긴 표시 이름은 줄임표/툴팁을 사용하고 원본 정밀도는 Advanced에 보존한다.

## 실험의 의미

Frame What-if는 저장된 해당 프레임 Snapshot에서 실제 C++ `evaluateDetection`을 재평가한다. 현재 지원 Override는 `vehicle_min_score`뿐이다. Association/KF를 진전시키거나 실제 Tracking 성공을 의미하지 않는다. Snapshot이 없으면 상세 Replay가 필요하다.

Dataset replay는 항상 첫 프레임부터 Detail end까지 기존 C++ Tracking 상태를 형성한다. Detail start~end에 상세 진단을 저장한다. Full replay는 원본 bag 전체를 실행한다. 실험마다 독립 Override와 결과/출처를 저장하며 운영 YAML은 수정하지 않는다. 저장 실험은 Sidebar → Experiments에서 다시 열거나 우클릭하여 해당 실행만 취소한다. 저장 What-if를 열면 해당 Dataset/Run/프레임으로 정렬한다.

## 데이터 보호와 범위

Schema 4의 manifest/index/bin 및 원본 bag은 읽기 전용으로 사용한다. Not Prepared는 자동 생성하지 않는다. 기존 Z 데이터셋 생성 CLI를 중단하거나 변경하지 않는다. 점군 LOD·색·크기 변경은 렌더링에만 적용한다. 운영 Perception/Crane/Redis 정책을 바꾸지 않는다.

실제 Jetson 기록은 없어 Not Available이다. 좌우 바탕 점군/초기 Cluster는 저장된 동일 Dataset이다. 변경된 Full 전처리 형상은 재구성하지 않으며 evidence 차이로 표시한다. 같은 숫자 Track ID가 물리적 차량 대응의 증거는 아니다. 모든 Ownership 경쟁이나 최종 선택 사유가 계측된 것은 아니며 미기록 원인은 만들지 않는다.

리디자인 검증은 실제 20261023 프레임 80·Track 6과 저장 결과를 사용했다. UI 검증을 위해 bag/Replay를 반복 실행하지 않았다. 화면과 검토 보고서: `/home/sjjin/construct_ws/general_debug_lab/UI_REDESIGN_KO.md`.


## BEV 객체 중심 탐색

- 장면의 **Fit**은 Fit Selected다. 선택 Cluster 실제 Point Bounds를 우선하며 Track은 Observed Cluster → 기록 Geometry/Prediction으로 표시 근거를 안내한다. ROI는 포함하지 않는다.
- Fit 옆 화살표에서 Observed / Current / Reliable / Track+Prediction 기준, Fit Visible, Fit ROI를 선택한다. Fit Visible의 ROI 포함 여부도 별도 설정이다.
- **Follow**는 선택한 한쪽 Run의 Track을 프레임 이동에 따라 따라가며 Zoom을 유지한다. 수동 드래그하면 해제된다. 포커스 아이콘은 카메라 중심만 이동한다.
- Layers 팝오버의 Points / Objects / Tracks / Context 필터는 렌더링에만 적용된다. Show All / Focus Only / Reset을 제공한다. 현재 inspect export에 없는 Background·Ignore·Unknown point 등은 이유와 함께 비활성화한다. ROI non-foreground를 Background로 분류하지 않는다.
- Workspace별 `view_settings.json`에 Layer/Focus Only/Fit 기준을 저장한다.
- Focus Only는 실제 초기 Cluster owner 또는 검증된 현재 Feature identity만 사용한다. 소속을 확인하지 못한 Track에는 가까운 Point를 붙이지 않고 모델만 표시한다.
- 하단 **Track lifecycle**은 선택 Track의 실제 관측/예측/누락 카운터/DORMANT/명시된 재획득/삭제 기록을 펼친다. Canonical LOST가 아닌 missed counter임을 구분한다. 구간 클릭으로 이동하고 드래그로 Detail replay 범위를 고른다. All tracks로 전체 이벤트로 돌아간다.
- PNG Export의 schema2 selection.json에는 Layer·selected object·Run·Frame·Fit·Follow·Camera transform과 실제 렌더링된 Point/Cluster/Track IDs를 기록한다.

실제 화면과 필터 정의·제한: `/home/sjjin/construct_ws/general_debug_lab/BEV_NAVIGATION_KO.md`.

## Crane 진단 모드 (Oct24 준비 데이터)

같은 Qt Window의 General/Crane 선택으로 전환한다. General의 기존 Explore/Compare/Tune은 유지된다. Crane은 현재 C++ 결과를 timestamp로 연결한 읽기 전용 offline 탐색이며 새 ROS 노드/웹 서버를 시작하지 않는다.

현재 workspace의 메시지/라이브러리 혼합을 피하려면 검증한 격리 환경과 state를 사용한다.

```bash
source /home/sjjin/construct_ws/log/crane_qt_motion_20261025/environment.bash
python3 /home/sjjin/construct_ws/src/cx128s2_bev_perception/test/general_debug_lab/desktop.py \
  --state /home/sjjin/construct_ws/log/crane_qt_motion_20261025/qt_state \
  --analysis-mode Crane
```

`--crane-data /path/to/prepared/evaluation`로 준비된 평가 폴더를 지정할 수 있다. 이번 adapter는 Oct24 Raw cache·기록 TF·기존 A 및 새 B 결과 묶음을 사용한다. 다른 bag의 자동 준비 도구는 아니다. 프레임 탐색 중 C++ 전체 replay를 실행하지 않는다.

XY/XZ/YZ/3D, A/B, object picker·Inspector, layer 토글, frame/time 이동, 13개 북마크, timeline 클릭·시간 확대, PNG/선택 확대/전후 3프레임 결합/북마크 일괄 export를 지원한다. Export JSON에는 센서 timestamp, 설정 YAML 내용/hash, 카메라와 실제 C++ 진단 값을 보존한다. 기본 export는 평가 폴더의 `qt_exports/`다.

녹색은 confirmed DIRECT, 노랑은 HELD/미확정, 보라는 Virtual Hook, 빨강은 저장된 rejected center다. NONE은 미탐 정답이 아니다. rejected center 주변 Raw 강조는 문맥이며 검출기의 실제 소속 점이 아니다. Gap ROI와 모든 rejected proposal membership은 미기록이므로 만들어 표시하지 않는다.

Moving B는 0.5초 시작·1.5초 종료·2.5초 support timeout을 비교한 실험 결과다. 운영 기본은 비활성이다. 프레임 내 raw evidence와 최종 output은 서로 다른 신호로 표시된다. 시간 필터가 실제 구조물 대응을 수정했다는 뜻은 아니다.

General의 기존 saved Schema 4와 현재 source contract는 작업 전부터 일부 불일치했다. 원본 dataset 자동 재생성이나 계약 우회는 하지 않는다. 저장된 프레임/Run 보기와 새 C++ replay 검증 범위는 구분한다.

전체 변경·회귀·실제 Qt 캡처: `/home/sjjin/construct_ws/docs/crane_qt_motion_20261025.md`.

## Housing 추가 학습 비교 (2026-10-25)

Crane Run 선택에 `Housing A — baseline`, `Housing C — learned`가 있으면 이 쌍이 기본이다. 기존 Moving 비교 Run도 유지한다. `Previous Δ / Next housing Δ`는 Housing 상태·ID·확인 상태가 다른 프레임으로 이동한다. `Housing IDs` timeline의 RIGHT는 표시 숫자에 1,000,000을 더해야 실제 ID다. 이전 중심·Wire support/association, 실제 C++ Reference/Rejected 후보 Geometry를 Layers에서 선택할 수 있다.

활성 학습은 저층 표본 f2488 LEFT / f2500 RIGHT의 Reference 2개이며, 원래 8개를 수정하지 않았다. 4개 표본 trial은 상부 Root 혼동 의심 때문에 별도 Run으로 보관했다. 새 신경망이 아니라 현재 descriptor Reference 방식이다. 기존 확인 Track과 현재 paired Wire, 학습 관측 범위에서만 추가 모델을 사용한다. ID 재발급과 흔들림이 해결된 것으로 해석하면 안 된다.

실행/구간 CSV/실제 Qt 이미지/실패 사례: `/home/sjjin/construct_ws/docs/crane_housing_learning_20261025.md`.

## Housing 선택적 추가 학습 E (2026-10-25)

기존 D(learned 2개)와 별개인 E 모델은 `config/crane_housing_learned_selective_oct24.yaml`이다. 기존 8개와 D의 2개를 보존하고 실제 paired Raw가 있는 RIGHT f2774/f2935를 추가했다. 위의 이전 4-reference trial(고층 표본 포함)과 이번 E는 서로 다른 모델이다. C++ 검출/추적 정책과 운영 기본 설정은 바꾸지 않았다.

```bash
bash /home/sjjin/construct_ws/src/cx128s2_bev_perception/test/start_general_debug_lab.sh --analysis-mode Crane --crane-data /home/sjjin/construct_ws/log/crane_housing_selective_20261025/qt_data
```

이 데이터에서는 A/D/E 3패널이 자동 선택된다. `+ E`로 3번째 패널을 토글하고 Inspector에서 패널을 선택한다. `Housing loss events` 목록은 기존 A의 520건을 제공하며 5시점 이동으로 이전/손실/재검출의 현재 Raw와 실제 C++ 후보 Gate를 비교한다. Raw 선택 확대는 화면용이며 처리 입력을 바꾸지 않는다.

Oct24 전체에서 E는 D의 DIRECT 손실 0, RIGHT +105, 공백 224→150이다. ID 재발급은 그대로이고 단발 미확정은 RIGHT 31→32로 늘어 f3975를 의심 사례로 남겼다. 외부 물리 GT 없이 새 검출을 모두 정답으로 간주하지 않는다. 보고서·20장 실제 Qt 이미지·별도 실험 실행 명령: `/home/sjjin/construct_ws/docs/crane_housing_selective_learning_20261025.md`.
