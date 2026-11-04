# General 오프라인 Replay Cache

기존 `replay_enriched.cpp`의 동기 호출 방식과 `build_replay.py`의 ROS CMake 플래그/링크 재사용 방식을 패키지 `test/`로 옮겨 확장했다. 운영 노드는 캐시 파일을 읽지 않는다. Detection/Tracking은 생산 소스의 `finishGeneralFrame()` 단일 구현을 호출한다.

## 실행

워크스페이스가 이미 빌드된 ROS 2 Jazzy PC에서:

```bash
T=/home/sjjin/construct_ws/src/cx128s2_bev_perception/test
R=/home/sjjin/construct_ws/log/general_replay_cache_20261023
B=/home/sjjin/enterance_ros_bag/rosbag2_2026_10_23-05_26_42

# 기존 Full 실행. --fast는 무거운 내부 상태 해시를 생략한다.
"$T/run_replay.sh" full --bag "$B" --cache "$R/unused" \
  --output "$R/manual_full" --tool "$R/tool" --fast

# 새 경로에 한 번만 생성한다. 기존 캐시는 덮어쓰지 않는다.
"$T/run_replay.sh" cache-build --bag "$B" --cache "$R/manual_cache" \
  --output "$R/manual" --tool "$R/tool" --depth-cache

# 최초 공식 A/B 사용 전에 전체 상태 일치 확인
"$T/run_replay.sh" cache-verify --bag "$B" --cache "$R/manual_cache" \
  --output "$R/manual" --tool "$R/tool" --depth-cache

# 개발 반복. Depth/Ownership은 프레임마다 독립 작업 사본을 사용한다.
"$T/run_replay.sh" cache-replay --bag "$B" --cache "$R/manual_cache" \
  --output "$R/manual_cached" --tool "$R/tool" --depth-cache --fast

# 전체 상태 검증 1회 후 Full/Cache를 교대로 실제 10회 측정
"$T/run_replay.sh" benchmark --bag "$B" --cache "$R/manual_cache" \
  --output "$R/manual" --tool "$R/tool" --depth-cache --repeat 10
```

`--depth-cache`를 양쪽에서 빼면 Phase A다. Phase A는 초기 Depth를 생산 `buildDepth()`로 재구성한다. Phase B는 초기 Cluster Owner 기준 Depth만 저장하며, 매번 독립된 grid를 복원한 다음 생산 Visibility/Ownership/Tracking을 실행한다. Phase를 바꾸면 별도 캐시를 생성한다.

`--params`, `--background`, `--roi`로 다른 설정 경로를 지정할 수 있다. 기본 CPU affinity는 `12-17`이다. CPU가 적은 PC에서는 `--cores 0-5` 등 실제 사용 가능한 코어를 지정한다. 스레드 수와 코어 수를 맞춰 비교한다. 기본 오프라인 ROS domain은 73이며 `GENERAL_REPLAY_DOMAIN`으로 변경한다. 운영 서비스/Docker/Redis를 조작하지 않는다.

## 데이터와 동일성

`frames.bin`의 각 schema 4 레코드는 압축/원본 길이, 원본 SHA256, 필드 단위 바이너리 데이터다. 선택 옵션 `--compress`는 LZ4 기본 압축 무손실 압축을 사용하며 생성/재생/검증 모두 같은 옵션을 지정한다. 기본값은 압축 OFF이며, 압축은 I/O/CPU 실측에 따라 선택한다. 점군 양자화는 하지 않는다. 비압축 schema 1 캐시는 재사용하지 않는다. PointCloud2 전체 메타데이터 CDR(payload 제외), 전체 변환 XYZIRT, Compact→원본 index, TF basis/translation, TF fallback, Valid/ROI/무시/ROR 전후 index, background similarity, Cluster 순서/indices, Raw Box 이후 Feature, 처리 제어 상태를 포함한다. 빈 Cluster 프레임도 저장한다.

센서 기반 계산은 생산 코드와 동일하게 저장된 World 점군과 정확한 TF의 역변환을 사용한다. 원본 sensor XYZ 비트값/RGB/padding payload는 별도로 복제하지 않으며 원본 bag은 보존한다. Tracking 후반부는 원본 payload를 참조하지 않는다.

PointXYZIRT는 22바이트 필드 단위로 저장하며 native padding을 쓰지 않는다. 빌드에 시스템 Boost.PFR, nlohmann-json, OpenSSL 및 LZ4 개발 헤더/라이브러리와 libxxhash 런타임 라이브러리가 필요하다. 큰 C++ 구조체는 Boost.PFR로 모든 aggregate field를 직렬화한다. KF는 private initialized/state/covariance/innovation 전체를 읽기 전용 friend로 비교한다. 주소/포인터는 저장하지 않고 실제 대상 Feature를 비교한다.

검증 실행은 입력→FG→Cluster→Feature/Raw Box→초기 Depth→초기 Visibility→전체 Track/KF/Memory/FIFO→최종 Depth→실제 public arrays를 SHA256으로 비교한다. 부동소수점까지 허용오차 0의 비트 일치다. 공개 메시지는 생산 `publishObjects()`에서 생성하고 ROS 송신 직전에 오프라인 callback으로 가로챈다. 프레임 지역 Feature 포인터가 살아 있는 시점에 실행한다.

`*_evidence.jsonl`, `*_times.csv`, `*_tracks.csv`, `*_stats.json`, `verification.json`, `benchmark.json`이 생성된다. `--fast`는 정밀 해시를 생략하므로 공식 전체 상태 동일성 확인을 대신하지 않는다. `benchmark --reuse-reference`는 동일 manifest·실행파일 SHA256에 묶인 기존 성공 검증만 재사용한다. benchmark는 정밀 검증을 먼저 하고, 성능 반복에서는 동일 Track CSV도 확인한다.

## 무효화와 안전성

Manifest는 전체 생산 CPP/headers/config, 실제 링크 라이브러리, driver calibration YAML, 실제 Background/ROI 파일, 파라미터, 빌드 fingerprint/ABI, 입력 순서를 검사한다. 원본 bag은 canonical 경로, inode, 크기, 나노초 mtime와 metadata SHA256으로 식별한다. 각 캐시 레코드는 XXH3-128을 검사하며 정밀 모드에서는 SHA256도 검사한다. `--fast`는 강한 소스/설정 무효화를 유지하면서 레코드 손상 확인에 XXH3-128을 사용한다. 공식 `cache-verify`는 모든 레코드의 SHA256을 확인한다. CLI의 fast 재생은 동일 manifest·실행파일에 대한 정밀 검증 성공 표시가 있어야 허용한다. MCAP 전체의 콘텐츠 SHA256을 매번 읽지는 않는다.

전처리/설정뿐 아니라 현재는 Tracking CPP 변경도 보수적으로 무효화한다. 검증된 단계별 소스 의존성 분리는 후속 작업이다. 강제 재사용 옵션은 없다. 불일치/손상/불완전 manifest는 오류 코드 2로 거부한다. 캐시가 없으면 `full`을 명시적으로 사용하거나 새 경로에 재생성한다.

빌드 스크립트는 기존 CMake test target의 flags/link line 및 기존 library object를 재사용한다. 정확한 dependency fingerprint가 동일하면 CPP 컴파일을 생략한다. CPP가 바뀌면 큰 TU 전체를 다시 컴파일한다. 생산 노드와 harness의 별도 TU 컴파일 자체를 제거하는 라이브러리 재구성은 하지 않았다.

## 처리 시퀀스와 한계

`--sequence FILE`은 원본 cloud의 1부터 시작하는 증가 순번 목록이다(한 줄 한 정수). Full/생성/재생 모두 같은 파일을 지정한다. 해당 순서로 원본 전처리부터 계산하므로 전체 캐시 단순 부분 추출과 다르다. 합성 시퀀스 시험은 Jetson의 실제 처리 이력 검증이 아니다.

v1 캐시는 General + 준비된 frozen static background + BEV connected component + object memory bank OFF에서만 허용한다. 기타 모드는 Full을 사용한다. TF/파싱 실패로 캐시 경계에 도달하지 않는 프레임은 조용히 누락하지 않고 미완성 캐시로 거부한다. 네 검증 bag의 프레임은 모두 경계에 도달해야 한다.

신선한 노드 상태에서 시작한다. bag 이전의 warm Track/KF/FIFO checkpoint는 복원하지 않는다. ROS worker/latest-only/실시간 TF 도착/Jetson 수치 차이/Redis end-to-end 검증은 별도다. 이 캐시가 실시간 성능이나 완전한 정답 GT를 입증하지 않는다.
