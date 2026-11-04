# Crane 움직임 유무 · 배경 분포 비교

현재 기본 `crane_motion_mode: scene_histogram`은 속도·방향을 구하지 않고, 배경 X/Y/Z 분포의 변화량 중 가장 큰 값으로 움직임 여부만 출력한다. Z만 보는 `z_histogram` 모드와 이전 상부 고정부 방식 `body_xy`도 선택할 수 있다. 기본 모드는 XY 이동 거리·속도 또는 양쪽 고정부 동시 관측을 요구하지 않는다.

## 기본값

config/crane_v2.yaml:

- crane_motion_mode: scene_histogram
- crane_hist_xy_bin_m: 0.5 (X/Y 분포 간격)
- crane_hist_bin_m: 0.10 (Z 분포 간격)
- crane_hist_start_score: 0.08 (낮출수록 민감)
- crane_hist_stop_score: 0.03
- crane_hist_lag_sec: 0.4
- crane_hist_start_confirm_sec: 0.2
- crane_hist_stop_confirm_sec: 0.8
- crane_hist_min_points: 1000

모두 시작 전용 파라미터다. 분포는 포인트 수로 정규화하며 경계에서 부드럽게 배분해 작은 bin 경계 노이즈를 줄인다. 각 축의 정규화된 분포 차이(Total Variation, 0~1)를 구하고 최대값만 판단에 사용한다. 기준 장면 대비 차이가 0.08 이상인 상태가 0.2초 이어지면 crane_moving=1. 최근 약 0.4초 전 장면 대비 차이가 0.03 이하인 상태가 0.8초 이어지면 0으로 바꾸고 기준 장면도 갱신한다.

Cargo·Holder의 현재/비교 프레임 영역 합집합을 양쪽 Raw에 동일하게 제외한다. Cargo만 상승·하강하거나 Holder가 움직이는 것을 직접적인 Crane 움직임 근거로 쓰지 않는다. 가림을 완전히 제거하거나 다른 동적 물체를 모두 분류하는 알고리즘은 아니다. 두 장면의 유효 배경이 부족하거나 크게 누락되면 새 움직임을 만들지 않고 마지막 값과 미관측 진단을 유지한다. 판단 시간은 센서 시각을 사용한다.

공개 출력은 기존 crane_moving 0/1뿐이다. Cargo FSM·길이·Holder·General 알고리즘에는 피드백하지 않는다. 디버그 토픽 /crane_beam/diagnostics의 hist_score, x_hist_score, y_hist_score, z_hist_score, hist_recent, background_points로 기준값을 검토할 수 있다. RViz에는 CRANE MOVE=MOVING/STILL을 표시한다.

## 검증 범위

통제된 배경 분포 변화에서 1, 안정 후 0, 결측 시 마지막 값 유지, Cargo/Holder만 Z 이동 시 0, 포인트 수 배율 변화만으로는 0을 확인했다. X·Y 양방향의 배경 위치 변화도 속도/방향 입력 없이 검출했다.

11/03 3.7초 원본 37프레임을 실제 현재 Node에서 처리했다. 배경 분포 변화가 작아 모두 0이었으며, 이는 해당 클립의 실제 물리 정지 여부를 판정한 GT가 아니다. Cargo·길이·상태·Holder 및 기존 Redis 필드의 유지 여부를 별도로 비교한다. 실제 크레인 이동이 확인된 구간의 민감도는 추가 확인이 필요하다.

이는 거친 장면 활동 신호이며 실제 크레인 이동의 정밀 측정/odometry가 아니다. 배경이 균일하면 이동해도 분포가 비슷할 수 있고, 다른 큰 물체나 가림 변화도 영향을 줄 수 있다. 운영 배포·서비스 재시작·Redis 실서비스 쓰기는 수행하지 않는다.
