# redis_comm (ROS 2 Jazzy Redis 통신 패키지)

로봇 시스템 및 관제 서버 간의 Redis Stream/List 통신을 담당하는 ROS 2 Python 패키지입니다.

---

## 1. 기본 설정 정보 (사무실 환경)

* **서버 IP (Host)**: `172.16.0.200` (로컬 테스트 시 `localhost`)
* **포트 (Port)**: `6379`
* **비밀번호 (Password)**: `1qaz`
* **대상 스트림 키**: `SP-OS-RW-R1`, `SP-OS-RW-R2`

설정 값은 `config/redis_params.yaml` 파일 또는 명령줄 인자로 변경할 수 있습니다.

---

## 2. 빌드 방법

```bash
cd /home/sjjin/construct_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select redis_comm
source install/setup.bash
```

---

## 3. 실행 방법

### 방법 0: 데스크톱 GUI 실행 (`redis_gui`, 권장)

`robotmanager` 스타일의 GUI 화면으로 Redis 연결, Key 목록 탐색, 스트림 메시지 실시간 조회 및 데이터/리스트 전송을 한 화면에서 수행할 수 있습니다.

```bash
# ROS 2 실행
ros2 run redis_comm redis_gui

# 또는 파이썬 직접 실행
cd /home/sjjin/construct_ws/src/redis_comm
python3 -m redis_comm.redis_gui
```

### 방법 A: 리스트 및 스트림 즉시 조회 (`redis_reader`)

Redis에 저장된 **Key 목록**과 **최신 스트림 메시지 리스트**를 터미널에 깔끔하게 출력합니다.

```bash
# 1. ROS 2 실행
ros2 run redis_comm redis_reader

# 또는 사무실 IP 직접 지정
ros2 run redis_comm redis_reader --host 172.16.0.200 --stream SP-OS-RW-R1 --count 10

# 2. 빌드 없이 파이썬으로 바로 실행하고 싶을 때
cd /home/sjjin/construct_ws/src/redis_comm
python3 -m redis_comm.redis_reader --host 172.16.0.200

# 3. 실시간으로 들어오는 메시지를 계속 모니터링할 때 (--follow)
ros2 run redis_comm redis_reader --follow
```

### 방법 B: 테스트 데이터 전송 (`redis_producer`)

데이터나 리스트를 Redis 스트림으로 보낼 때 사용합니다.

```bash
# 1회 일반 상태 데이터 전송
ros2 run redis_comm redis_producer --once

# 경유지 목록(Waypoint List) 전송
ros2 run redis_comm redis_producer --send-list --once

# 2초마다 지속 전송
ros2 run redis_comm redis_producer --interval 2.0
```

### 방법 C: ROS 2 주기적 통신 노드 실행 (`redis_node` / Launch)

주기적으로 Redis를 조회하여 ROS 2 토픽(`/redis/stream_data`)으로 퍼블리시합니다.

```bash
# 노드 직접 실행
ros2 run redis_comm redis_node

# 또는 Launch 파일로 파라미터와 함께 실행
ros2 launch redis_comm redis_reader.launch.py
```

---

## 4. 파이썬 코드에서 라이브러리로 직접 사용하기

다른 노드나 스크립트에서 Redis 통신을 사용할 때:

```python
from redis_comm.redis_client import RedisCommClient

client = RedisCommClient(host='172.16.0.200', port=6379, password='1qaz')
if client.connect():
    # 1. 모든 키 목록 조회
    keys = client.get_all_keys("*")
    print("Keys:", keys)

    # 2. 스트림 메시지 리스트 5개 읽기
    messages = client.read_stream_list("SP-OS-RW-R1", count=5)
    for msg in messages:
        print("ID:", msg["id"], "Payload:", msg["parsed_payload"])

    # 3. 데이터(딕셔너리 or 리스트) 전송
    client.send_stream_data("SP-OS-RW-R1", {"status": "moving", "battery": 95})
```

