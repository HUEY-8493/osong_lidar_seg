# Crane 본체 좌우 이동 출력 · 선택적 body_xy 모드

현재 기본값은 scene_histogram이다. 이 문서의 XY 속도/변위 조건은 crane_motion_mode=body_xy를 명시했을 때만 적용된다. 기본 방식은 crane_v2_scene_motion.md 참고.

V2의 crane_moving=0 고정을 제거하고, 이전 Crane 설정의 좌·우 상부 고정부 ROI를 사용한 독립 XY 판단을 연결했다. Cargo/Hook 검출·Ground·LIFTING/SUSPENDED/LOWERING FSM에는 영향을 주지 않는다. 동일한 기존 ROS bool을 Bridge/Manager가 그대로 Redis crane_moving 0/1로 전달한다.

## 기본 기준

config/crane_v2.yaml의 시작 전용 파라미터:

- crane_xy_enabled: true
- crane_xy_start_m: 0.04m. 초기/정지 기준점에서 양쪽 고정부가 각각 이 이상 이동해야 한다.
- crane_xy_min_speed: 0.03m/s. 최근 실측 시간창의 XY 속도 기준.
- crane_xy_start_confirm_sec: 0.2초. 양쪽 방향·변위 일치가 연속 확인돼야 한다.
- crane_xy_stop_span_m: 0.015m. 양쪽 고정부 XY 관측 범위가 이 이내로 crane_xy_stop_confirm_sec=0.8초 유지되면 정지로 갱신하고 기준점을 새로 잡는다.
- crane_xy_min_points: 각 ROI의 실측점 15개 이상.
- crane_xy_left_root_roi: [-1.25, -0.55, -0.25, 0.35, 4.95, 5.55]
- crane_xy_right_root_roi: [0.72, 1.42, -0.25, 0.35, 5.00, 5.60]

ROI 순서는 xmin,xmax,ymin,ymax,zmin,zmax이며 world 좌표다. 기존 calibrated fixed_root ROI를 가져왔다. 이전 fixed_root의 8cm·0.12m/s 기준보다 시작을 민감하게 설정했다. 이는 이전 V2에서 낮춘 임계값이 아니라, 0 고정 상태였던 V2에 새로 연결한 별도 판단의 기본값이다.

## 관측 정책

각 ROI의 실측 XYZ 중앙값을 사용하고 현재 Cargo Raw 및 관측/유지 Hook housing 근처 점을 제외한다. 양쪽 고정부의 변위와 속도가 같은 방향이고 변위 차이가 허용 범위 이내여야 한다. 최근 Z 변화가 큰 경우에는 시작 근거로 사용하지 않는다. Hook/Cargo만 상승하거나 한쪽만 흔들리는 것을 본체 좌우 이동으로 직접 연결하지 않는다.

고정부 관측이 부족하거나 sensor gap이 생기면 임시 판단 창과 기준점을 초기화하되, 마지막 moving 값은 유지한다. 새 이동을 추정하지 않으며, 양쪽이 다시 관측되면 정지/이동을 판단한다. 독립 진단의 BODY_ROOTS_UNOBSERVED_HOLD와 RViz HELD/UNOBSERVED가 이 상태를 구분한다. Redis에는 새 공개 Key를 추가하지 않았다.

기존 RViz marker 토픽에서 하늘색 구가 현재 관측 고정부 중앙값이며 CRANE XY=MOVING/STILL 또는 HELD/UNOBSERVED를 표시한다. `/crane_beam/diagnostics`의 crane_xy와 root_points로 관측 부족을 확인할 수 있다.

## 범위와 한계

상부 고정부 ROI가 실제 본체에 정렬돼 있다는 기존 설치 가정을 사용한다. 다른 구조물이 ROI를 채우거나 부분 가림으로 중앙값이 함께 이동하면 본체 이동과 혼동할 수 있다. 실제 고정부가 ROI 밖으로 나가거나 Hook과 겹쳐 제외되면 관측 부족으로 유지된다. 정밀 ego-motion/odometry를 구현한 것은 아니다.

사용자의 직전 테스트 생략 요청에 따라 Rosbag/동작 테스트는 실행하지 않았다. 소스 빌드만 확인했다. 운영 서비스 재시작과 Redis 실서비스 쓰기는 하지 않았다. 적용은 운영 작업공간에서 새로 빌드한 다음 Crane 프로세스를 재기동해야 한다.
