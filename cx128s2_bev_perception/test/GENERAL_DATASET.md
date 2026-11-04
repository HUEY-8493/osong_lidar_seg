# General 영구 오프라인 데이터셋

운영 ROS 노드와 분리된 개발 CLI다. 원본 bag, 운영 서비스, Redis를 수정하지 않는다. 기본 저장 위치는 `/home/sjjin/construct_ws/general_replay_dataset`이다. 원본 bag 경로를 반복 입력할 필요가 없다.

```bash
D=/home/sjjin/construct_ws/src/cx128s2_bev_perception/test/general_dataset.py

# 최초 준비: 지정된 네 bag을 순차 처리, Full과 전체 상태 비교
python3 "$D" prepare --bag all --verify

# 준비 상태, 프레임 수, 디스크 용량
python3 "$D" list

# 현재 소스를 자동 빌드하고 기존 데이터셋으로 테스트
python3 "$D" test --bag 20261023 --label baseline
# 이후 KF / Association / 상태 / Geometry Memory / 최종 차량 판단 수정 후
python3 "$D" test --bag 20261023 --label candidate
python3 "$D" compare --bag 20261023 --baseline baseline --label candidate

# 1번부터 90번까지 상태를 형성하고 80~90번 상세 이미지 생성
python3 "$D" inspect --bag 20261023 --frames 80:90 --label candidate
# 시간 범위도 첫 cloud 기준 초로 조회 가능 (범위의 모든 프레임 PNG)
python3 "$D" inspect --bag 20261020 --seconds 254:256 --label candidate
# 부분 테스트는 앞에서부터 warm-up; 선택 양끝 프레임 PNG와 전체 CSV
python3 "$D" test --bag 20261023 --frames 80:90 --label section

# 현재 알고리즘의 Full / Dataset 전체 비트 동일성 검증
python3 "$D" verify --bag 20261023

# 실제 10회 반복; 1/5/10회 누적치 + Full 비교 (필요 시 명시적으로 실행)
python3 "$D" test --bag 20261023 --label performance --repeat 10 --full-reference

# 전처리 변경 시 해당 bag만 명시적으로 재생성. 기존 데이터는 archived로 보존.
python3 "$D" prepare --bag 20261023 --regenerate --verify
```

Bag 키: `20261023`, `20261020`, `20260729`, `20261007`, `all`. 기본 코어 12–17은 `GENERAL_REPLAY_CORES=0-5` 등으로 변경한다. ROS domain 기본 73, `GENERAL_REPLAY_DOMAIN`으로 변경한다. 명령은 한 root 안에서 동시에 실행하지 않는다. Ctrl+C는 자식 엔진까지 종료한다.

## 저장 데이터와 불변성

각 bag 디렉터리의 `manifest.json`, `frames.bin`, `frames.index`를 보존한다. 바이너리는 lossless LZ4 / 필드 단위 schema 4이며 원시 World XYZIRT 전체(점당 22바이트), compact→원본 Point Index, 실제 TF/header/PointCloud2 필드 메타데이터, Valid/ROI/무시/FG/ROR 전후 인덱스, Background 유사도, 초기 BEV Cluster/indices, Raw Box 이후 Feature, 초기 cluster owner 및 pre-remap Depth를 저장한다. Track 상태는 데이터셋에 고정하지 않는다. 원본 PointCloud2 padding/RGB/센서 XYZ 비트열은 복제하지 않으며 원본 bag을 보존한다. 생산 후반 경로는 저장된 World 점군과 TF 역변환을 사용한다.

`frames.index`는 frame/input_sequence, 실제 timestamp, 상대 시간, 바이너리 offset/길이, 점군/cluster 수, 처리 여부, 실제 적용된 azimuth 간격 및 ray elevation 간격을 가진다. `inspect`는 해당 offset으로 실제 한 프레임만 읽어 형상을 조회한다. 시각화의 Track 위치는 별도 앞부분 warm-up 결과를 겹친다. 해당 프레임에서 빈 Track으로 시작한 결과를 정상 추적으로 표시하지 않는다.

초기 Depth에는 angular bin 구조, occupied indices, nearest/farthest range, 실제 nearest World 위치/cluster owner/count가 저장된다. 각 재생은 새 작업 grid를 복원하고, Track ID remap / predictVehicleSurface / memory visibility / Association / 가림 판정은 현재 생산 코드로 다시 계산한다. 실제 angular returns 및 ring profile은 보존된 원시점+TF로 조회한다. 작은 초기 angular/ring profile 추정 및 static ROI cache 초기화는 현재 코드에서 다시 실행하되, 이 함수 변경은 prefix 의존성에 포함된다.

준비 완료 데이터 파일은 읽기 전용(0444), bag 디렉터리는 0555로 설정한다. 결과는 별도 `results/<bag>/<label>/`에 기록한다. 재생은 bag Reader를 만들지 않으며 원본 bag stat/metadata 조회도 하지 않는다. 원본을 읽는 명령은 `prepare`, Full을 수행하는 `verify` 및 명시적 `--full-reference`뿐이다. 실패한 불완전 데이터는 자동 삭제하거나 덮어쓰지 않는다. 확인 후 `--regenerate`를 사용한다.

## 실제 의존성 분리

`dataset_dependencies.py`는 tree-sitter C++ AST의 함수 정의를 사용한다. 전처리 진입 함수의 호출 이름을 재귀적으로 따라가며, 동명 overload는 모두 포함한다. 전처리와 공유되는 helper는 변경 시 무효화된다. 전처리 함수 목록과 source별 digest를 manifest `contract.prefix`에서 확인할 수 있다. 전체 CPP SHA는 영구 데이터셋의 호환성 판정에 사용하지 않는다.

- 재사용: `updateTracks`, 최종 `evaluateDetection`, 상태 처리, KF 구현, Track-dependent `evaluateBox` 및 이를 사용하는 후반 로직의 함수 본문 변경.
- 재생성: parsing/TF/crop/ROI/FG/ROR/BEV/Feature/Raw Box/buildDepth/초기 angular profile 및 관련 helper 변경.
- 명확하지 않은 구조/초기화 변경: serialized aggregate 선언, 클래스 멤버, 생성자, 새 함수 추가/삭제, 컴파일 옵션/ABI/의존 라이브러리 변경은 보수적으로 거부할 수 있다. 실패 시 `*_expected_contract.json`과 manifest를 비교한다. 강제 재사용 기능은 없다.
- 파라미터: 전처리에서 소비하는 멤버의 get_parameter 매핑과 prefix 관련 설정을 비교한다. 후반 전용 `vehicle_min_score` 등은 재사용 가능하다. ROI/Background/driver calibration 파일 내용은 별도로 고정한다.

tree-sitter 및 tree-sitter-cpp는 workspace `.general_replay_python`에 설치했다. 다른 PC에서는 다음으로 준비한다.

```bash
python3 -m pip install --target /home/sjjin/construct_ws/.general_replay_python tree-sitter tree-sitter-cpp
```

현재 Visibility 구현은 운영 공유 라이브러리를 바꾸지 않고 오프라인 object로 직접 빌드한다. 따라서 Visibility 소스를 수정한 뒤 오래된 운영 빌드 라이브러리로 시험하는 문제를 피한다. 빌드 fingerprint는 전체 코드 변경을 추적하며, 데이터셋 호환성 fingerprint는 prefix만 추적한다.

## 결과와 검증

`*_tracks.csv`, `*_trace.jsonl`(실제 association source, detection rejection, candidate decision, LOST/DORMANT/lifecycle/FIFO/status), `*_times.csv`, `*_stats.json`, `results.json`, `timeline.png`를 저장한다. 내부 Association의 모든 pair별 탈락 조건을 새로 계측한 것은 아니므로 기존 코드가 제공하지 않는 거절 이유는 만들어내지 않는다. 최초 공개 감지, 실제 VEHICLE 관측 frame-ID 수, ID별 최대 단절, CPU 평균, 프레임 시간 mean/p95/p99/max, peak RSS를 기록한다.

`verify`는 Full과 Dataset의 입력/World/TF/FG/Cluster/Feature/초기·최종 Depth/초기 Visibility/모든 Track 및 KF 공분산/Memory/FIFO/공개 출력의 비트 단위 digest를 비교한다(허용오차 0). 최초 불일치 frame/path를 기록한다. 후보 알고리즘을 수정한 뒤에는 현재 Full과 Dataset을 비교하는 것이며, 이전 baseline과 같아야 한다는 의미가 아니다. A/B 차이는 `compare`가 별도로 보여준다. 데이터 레코드는 SHA256+XXH3, 빠른 test는 XXH3, 인덱스는 SHA256으로 검사한다.

PNG는 실제 World 점군, ROI 경계, Cluster ID/점수, 센서 angular return, 저장된 초기 depth와 실제 ring profile을 보여준다. Track의 실제 관측은 원, prediction-only는 x로 구분한다. 예측/기억을 실제 관측으로 계산하지 않는다. 범위 inspect는 현재 알고리즘을 1번부터 재생하여 상태를 만든다.

Fresh node 시작이며 bag 이전 warm Track checkpoint는 복원하지 않는다. frozen static BG/General/BEV CC/object memory bank OFF만 지원한다. Jetson 실기, ROS latest-only worker 이력, Redis end-to-end 검증은 별도다. 여러 반복 수치는 실제 완료된 실행만 보고하며 최초 준비/검증 비용은 반복 실행 시간에서 구분한다.

## 다른 폴더의 bag 준비

`--bag-dir`는 metadata.yaml을 가진 하위 bag 폴더들을 찾는다(단일 bag 경로도 가능). 외부 폴더에는 별도의 `--root`를 권장한다. prepare가 root/datasets.json에 원본 경로를 등록하므로 이후에는 --root와 bag 이름만 지정한다. list는 등록 파일을 쓰거나 전처리하지 않는다.

```bash
D=/home/sjjin/construct_ws/src/cx128s2_bev_perception/test/general_dataset.py
R=/home/sjjin/construct_ws/general_replay_dataset_osong
python3 "$D" prepare --bag-dir /mnt/z/osong/oosong_record/enterance --bag all --root "$R" --verify
python3 "$D" list --root "$R"
python3 "$D" test --root "$R" --bag rosbag2_2026_10_23-05_26_42 --label current
```

prepare --verify는 각 bag을 한 번 생성하고 Full/데이터셋 동일성을 한 번 비교한다. 5/10회 반복 측정은 수행하지 않는다. 특정 bag만 준비하려면 --bag all 대신 해당 폴더 이름을 사용한다.

첫 점군 전에 TF 메시지가 하나도 없는 bag은 같은 bag의 `/tf_static`을 먼저 읽어 초기 변환을 확보한다. 정적 TF만 선행 로드하며 `/tf`의 동적 변환은 기록 순서를 유지한다. Full과 cache-build가 같은 경로를 사용한다. bag에 필요한 정적 변환 자체가 없으면 여전히 실패하며 임의 보정을 만들지 않는다. 실패한 불완전 데이터셋을 재시도할 때는 `--bag <해당 이름> --regenerate --verify`로 그 bag만 지정한다. 기존 실패 디렉터리는 `.archived.<시간>`으로 보존된다.
