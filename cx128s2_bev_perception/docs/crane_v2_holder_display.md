# Sheave Holder 실측 표시

11/03 Bag의 좌/우 Holder는 기존 V2에서 모두 37/37 DIRECT로 검출됐지만, RViz에는 실제 Housing 대신 아래쪽 가상 Hook 점만 보였다. 사용자가 좌 Z≈2.25m / 우 Z≈2.57m 구조물이 Holder임을 확인했다.

검출 조건·점수·추적·가상 Hook 위치를 바꾸지 않고, 기존에 선택한 Holder Raw와 바운드를 시각화한다. 초록 윤곽 및 LEFT/RIGHT HOLDER DIRECT/HELD 라벨, Holder에서 가상 Hook으로 이어지는 선을 기존 marker에 추가했다.

기존 `/crane_hook/left_body` 및 `/crane_hook/right_body`에 현재 실측 Holder XYZ PointCloud를 world 좌표와 원본 센서 시각으로 발행한다. 구독자가 있을 때만 cloud를 구성한다. 미관측/HELD에서는 현재 Raw를 만들지 않고 빈 cloud를 보낸다. 기존 Cargo POINTS marker와 상태 라벨을 섞거나 변경하지 않는다.

Crane RViz 프리셋의 Left/Right Hook Body 표시를 기본 ON으로 했다. General RViz는 바꾸지 않았다. 적용 시 perception과 lidar_bringup을 함께 빌드하고 Crane/RViz를 재실행한다.

검증: 실제 새 Node에서 같은 원본 37프레임을 처리해 기존 Cargo 상태·길이·centroid·tracking·포인트 수·Redis payload 및 Cargo 박스가 모두 동일했다. 좌우 Holder cloud 포인트 수가 기존 검출값과 전 프레임 일치하고, 중앙값으로 계산한 Holder 위치 차이는 0m였다. Holder Raw·라벨·센서 시각·HELD empty cloud 통제 테스트가 통과했다. 운영 서비스 재시작과 Redis 실서비스 쓰기는 수행하지 않았다.
