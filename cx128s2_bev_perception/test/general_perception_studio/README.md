# General Perception Studio

General용 네이티브 C++17 / Dear ImGui Docking / GLFW / OpenGL / ImPlot 프로그램이다. HScore Studio 소스는 이 환경에 없어 재사용하지 않았다. 요청된 기술 스택으로 독립 구현했다. 경로 탐색/Controller 코드는 포함하지 않는다.

## 실행 / 빌드

```bash
bash /home/sjjin/construct_ws/src/cx128s2_bev_perception/test/start_general_debug_lab.sh
# 동일한 네이티브 런처
bash /home/sjjin/construct_ws/src/cx128s2_bev_perception/test/start_general_perception_studio.sh
# Qt reference / fallback, 별도 실행
bash /home/sjjin/construct_ws/src/cx128s2_bev_perception/test/start_general_debug_lab.sh --qt
```

같은 state를 Qt와 Studio가 동시에 사용하지 못하도록 잠근다. 열린 Qt를 먼저 닫는다. `GENERAL_DEBUG_LAB_STATE`로 state를 선택할 수 있다. 원본/캐시는 자동 재생성하지 않는다. 기존 state의 실험·프레임 LOD 캐시를 그대로 읽는다.

최초 런처는 기존 `build_debug.py`로 Replay를 확인하고, 읽기 전용 상주 Reader와 CMake UI를 빌드한다. 이후 변경 없는 실행은 재사용한다. 고정된 공식 소스를 state/native_deps에 가져온다.

- ImGui v1.91.9b-docking: `4806a1924ff6181180bf5e4b8b79ab4394118875`
- ImPlot v0.16: `18c72431f8265e2b0b5378a3a73d8a883b2175ff`
- 시스템 의존성: C++17 compiler, CMake, GLFW3, OpenGL, libpng, nlohmann/json; 기존 ROS/Replay/Python 의존성 유지.

UI만 빌드:

```bash
cmake -S /home/sjjin/construct_ws/src/cx128s2_bev_perception/test/general_perception_studio \
  -B /home/sjjin/construct_ws/general_debug_lab/native_build \
  -DSTUDIO_DEPS=/home/sjjin/construct_ws/general_debug_lab/native_deps \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /home/sjjin/construct_ws/general_debug_lab/native_build -j 4
```

`--dataset-root /absolute/root`는 custom0, custom1 등으로 추가한다. local과 Z root는 기본 등록된다. `--width 1920 --height 1080`을 지원한다. 실험/로그/설정/PNG는 state에만 기록한다.

## 구성과 재사용

`main.cpp`는 윈도우, Docking, VBO/FBO 렌더링, 카메라, Picking, ImPlot, Inspector와 실험 조작을 담당한다. GUI는 ROS 노드가 아니며 ROS publisher를 만들지 않는다. `worker.py`는 소켓 없이 stdin/stdout JSONL로 연결되는 상주 자식 프로세스다. 기존 `Lab`의 조회/실험/취소/비교/결과 저장을 호출한다.

`build_reader.py`는 기존 진단 실행파일 소스의 Schema 4 **inspection 함수**를 stdin 루프로 호출하는 별도 C++ 어댑터를 생성한다. 기존 Replay 실행파일과 소스는 수정하지 않는다. 아직 LOD 캐시가 없는 프레임도 같은 Reader PID로 읽는다. SHA/XXH 레코드 검사, serializer/자료형 크기 검사와 기존 LOD 함수를 그대로 사용한다. 데이터셋 파라미터를 바꾸거나 전처리를 다시 계산하지 않는다. 캐시된 프레임은 Reader를 실행하지 않는다.

`general_debug_lab/view_evidence.py`는 Qt에서 사용하던 순수 표시 함수들을 옮긴 공통 모듈이다. Qt와 Studio가 관측 여부, 검증된 point/cluster 소속, 카메라 경계, 실제 기록 Lifecycle을 함께 사용한다. Qt 클래스와 `QRectF` 변환은 기존 `bev_view.py`에 남는다. Perception/Association 알고리즘을 추가하지 않았다.

## 기능 대응

| 기존 기능 | 네이티브 구현 / 연결 |
|---|---|
| Dataset / 추가 root | Header Dataset 선택, --dataset-root, 기존 metadata/index 검증 |
| Frame / 시간 / Play / Step / 속도 | Timeline, Space, 실제 index timestamp 조회 |
| BEV / LOD / Point 크기 | World XY, OpenGL VBO/FBO; 프레임·필터 변경 시 업로드 |
| Cluster / Track 선택 / Hover | 실제 LOD compact index와 Cluster ID, Track의 기록 위치 |
| Fit Selected / Visible / ROI | 공통 view_evidence의 선택 경계; 명시적 ROI fit |
| Focus / Follow | 기존 display_position, anchor Before/After, zoom 유지; 수동 pan 시 Follow 해제 |
| KF / 실제 관측 / Geometry / Memory | 원형 관측·보라색 예측 X·현재 box·점선 Memory; 별도 Inspector |
| Layer / 상태 필터 / Focus Only | 표시 전용. Rejected와 Unknown 구분. 없는 의미는 비활성화 및 사유 표시 |
| Explore / Compare / Tune | DockSpace 프리셋, 동기화 World camera; 같은 Dataset/Frame/선택 유지 |
| Gate / Decision / Raw | 실제 C++ gate와 source/line tooltip, FAIL 우선, NOT_EVALUATED 유지 |
| FOV / Depth | 기존 Visibility 수치, 실제 저장 Depth angular return scatter, 모델/실제 구분 |
| Timeline / Lifecycle / Events | 기존 trace·CSV와 공통 lifecycle; 관측/Prediction/미관측 counter/DORMANT/명시적 Reacquire·Removed/FIFO |
| ImPlot metrics | 기록된 X/Y/속도/공분산/Geometry/Score/Angular/실제 Gate 값/처리시간. 클릭 seek, 결측 비보간 |
| A/B Run / 공통 필드 축약 | 기존 Lab.compare, timestamp 검증, 차이 있는 Track 필드만 펼침 |
| Parameter Lab | 기존 지원 파라미터 목록, current/JSON override/단위/기본값/함수/호환성 metadata tooltip |
| Frozen Detection What-if | 기존 snapshot 및 vehicle_min_score 실행, 기존 결과/실험 복원 |
| Dataset Replay / Full Replay | 기존 Lab.start/command/work, 비동기 실행, 기존 호환성 검사 유지 |
| 실험 저장 / 복원 / 취소 / 진행률 | 기존 experiment.json, trace 진행률, 기존 결과를 그대로 복원 |
| PNG / 선택 정보 | 실제 GL framebuffer PNG + Dataset/Frame/Track/Run/camera/layer/manifest 정보 JSON |
| Layout / Theme / Focus | imgui ini Docking 복원, Warm light / Neutral dark, Workspace UI scale, F11 Focus View |
| Qt 참조 및 기존 Crane 분석 UI | --qt fallback 보존. 새 네이티브 창은 General workspace 전용 |

Dock panel의 탭은 드래그로 이동/분할할 수 있다. Workspace 메뉴로 패널을 선택하고 프리셋을 복원한다. Multi-Viewport는 현재 플랫폼의 창 밖 분리 검증을 하지 않아 비활성화했다. F11에서 돌아오면 해당 기본 프리셋을 다시 구성한다.

## 실제 검증 및 한계

`/home/sjjin/construct_ws/general_debug_lab/validation/studio_release`에 실제 네이티브 화면 14장이 있다. Qt 비교 화면은 `validation/qt_reference_frame80_track6.png`다. 상세 결과는 state/NATIVE_STUDIO_KO.md, native_backend.json, studio_final_checks.json, persistent_reader.json을 참고한다.

- local 20261023 / frame 80 / Track 6: 135,267 original points, 22,075 LOD, 15 clusters, height_min 0.05 / 1.8 FAIL.
- 기존 Qt와 Frame 데이터·Trace·CSV·KF/Geometry/Gate·Lifecycle 및 모든 Track의 네 가지 Fit 경계값 일치.
- 기존 Full Replay로 Z의 131프레임 완료. 같은 바이너리로 새 snapshot의 What-if 완료 및 저장/복원 일치.
- 긴 local 20261020 bag의 frame 2500 열기 완료.
- 즉시 취소 시 running으로 되돌아가는 기존 Python 작업 준비 경합을 lock으로 수정하고 재검증했다. 감지/추적 계산 변경은 없다.

현재 저장 Dataset의 prefix와 최신 소스가 다른 경우 **보기는 가능하더라도 Dataset Replay는 기존 호환성 검사로 거절된다.** 실제 실험에서 이 거절을 확인했다. 이를 성공으로 집계하지 않으며 자동 재생성/강제 우회하지 않는다. Full Replay에는 원본 bag 경로가 존재해야 한다. 원본이 Z에 있는 경우 Z Dataset을 선택한다. 오래된 binary snapshot도 기존 same-binary 검사에 따라 새 What-if에 사용할 수 없으며 저장된 과거 What-if 결과 조회는 가능하다.

프레임 읽기 중 UI는 별도 프로세스/스레드로 계속 동작하지만, cold Z 프레임의 압축 해제·전체 JSON→기존 LOD 생성은 수 초가 걸릴 수 있다. 이번 작업에서 모든 bag을 새로 만들거나 전체 프레임의 화면 캐시를 미리 생성하지 않았다. 기록에 없는 exact Background/Ignore/Unknown point semantic 및 canonical LOST는 여전히 Not Available이다. Jetson 실제 기록 비교, Multi-Viewport, Crane의 ImGui 분석 화면은 구현/검증하지 않았다. 기존 Crane 기능과 Qt 분석 경로는 보존했다.

## NAS 프레임 Buffer

상단 `Buffer` 메뉴에서 RAM 캐시와 미리 읽기를 제어한다. 기본 표시 프레임 RAM 한도는 512 MiB이며 32–2048 MiB로 바꿀 수 있다. 별도 Run/Trace RAM 캐시는 128 MiB 한도다. 실제 Python 객체 크기를 집계하고 LRU로 비워 한도를 유지한다. 이 수치는 캐시 예산이며 전체 프로세스 RSS 제한은 아니다.

- `Auto prefetch next NAS frames`: Z/추가 root에서 다음 2프레임 자동 준비. 1–16으로 변경 가능.
- Timeline의 Detail start/End를 정하고 `Preload selected range`로 해당 구간을 준비한다.
- `Stop preload`: 대기 중 범위는 취소하고 진행 중인 한 프레임은 마무리한다. 현재 프레임 요청을 다음 미리 읽기보다 우선한다.
- `Clear RAM`: 자동 미리 읽기를 끄고 RAM을 비운다. 기존 로컬 LOD 파일은 보존한다.
- RAM 한도/자동 설정은 `state/buffer_settings.json`에 저장한다.

이미 준비한 화면 데이터는 기존 로컬 `state/frames/<manifest SHA>/`에도 남으므로 재실행 시 NAS의 원본 프레임을 다시 디코딩하지 않아도 된다. RAM 한도보다 큰 구간은 준비하면서 앞 프레임이 LRU로 빠질 수 있지만 로컬 표시 캐시는 남는다. 선택 범위가 클수록 로컬 표시 캐시의 디스크 사용량도 늘어난다.

완료된 Run/Trace를 RAM에 보관하고 metadata/run 목록의 반복 NAS 접근도 줄였다. metadata/Trace 서명은 최대 5초, Run 목록은 2초 주기로 확인하며 Dataset을 다시 열면 재검사한다. 재생 중인 원본/운영 계산을 캐시로 대체하지 않으며, 미리 준비하는 것은 표시용 프레임이다. CLI/Replay 데이터 생성은 실행하지 않는다.

실제 Z 20261007에서 80–83번 프레임 준비, RAM 한도 축소에 따른 제거, 취소, Clear RAM, 자동 다음 2프레임 준비, 설정 복원을 검증했다. 원본 dataset 파일의 크기/mtime은 그대로였다. 결과: `validation/frame_buffer.json`, `validation/buffer_auto.json`; 실제 UI: `validation/frame_buffer_ui/buffer.png`.

## Tracking Line 비교 (2026-10-26)

기존 `Timeline`은 선택 Track의 Before/After 실제 점군 연결, 공개 VEHICLE, Prediction-only, 기록된 LOST/DORMANT, 연결되지 않은 bounded 조각, 분류 탈락을 별도 Lane으로 표시한다. 없는 값은 빈 구간으로 남는다. `World XY history`는 실제 적용 Measurement와 KF 예측을 구분하며, World 원점 반경 50m 점선은 ROI와 다른 평가 경계다. 재관측 마커는 실제 관측 재개이며 canonical Reacquire와 같다고 가정하지 않는다. Run 사이 숫자 ID만으로 물리적 동일성을 보장하지 않는다.

Timeline/지표의 점 또는 World XY 궤적을 클릭하면 프레임, Before/After 선택, 해당 Track, BEV 실제 Cluster, KF/Geometry, Gate Inspector가 같이 갱신된다. `Center on current`와 `Use frame range`로 시간 범위를 맞춘다. 선택 객체 Fit은 전체 ROI보다 객체를 우선한다. 실제 관측은 실선, KF 모델은 끊긴 선이며 누락 구간을 보간하지 않는다.

`ImPlot metrics`의 `angular_bins`에는 C++가 해당 프레임에서 실제 계산한 Required Bins를 함께 그린다. VEHICLE 크기 점수는 `vehicle_size_score`이며 일반 출력 class score와 다르다. 현재 계산되지 않은 지표는 비어 있다. 관측 누락 때 Angular 값이 있으면 **현재 후보 검사** 값이며 수락된 관측이라는 뜻이 아니다. 구형 Trace에는 Measurement/LOST/Association 세부값이 없을 수 있어 KF 출력으로 대체하지 않는다.

검증 전용 비교 workspace 실행:

```bash
bash /home/sjjin/construct_ws/log/general_world50_round2_20261026/launch_validation.sh
```

이 workspace의 `before/dataset`, `after/dataset`은 완료된 전체 Replay 결과를 가리킨다. 원본 데이터셋을 바꾸지 않았다. `--validate-lines DIR`은 같은 네이티브 창에서 실제 ImGui 마우스 이벤트로 Timeline/XY 클릭, 프레임·Track·Gates·객체 Fit 연동을 검사하고 실제 GL 화면/JSON을 저장한다. 신규 서버나 별도 GUI를 실행하지 않는다.
