import argparse
import sys
import time

try:
    from redis_comm.redis_client import RedisCommClient
except ImportError:
    from .redis_client import RedisCommClient


def parse_args():
    parser = argparse.ArgumentParser(description="Redis Stream Producer for testing")
    parser.add_argument("--host", default="172.16.0.200", help="Redis server IP (default: 172.16.0.200)")
    parser.add_argument("--port", type=int, default=6379, help="Redis server port (default: 6379)")
    parser.add_argument("--password", default="1qaz", help="Redis password (default: 1qaz)")
    parser.add_argument("--stream", default="SP-OS-RW-R1", help="Target stream key (default: SP-OS-RW-R1)")
    parser.add_argument("--once", action="store_true", help="Send once and exit")
    parser.add_argument("--send-list", action="store_true", help="Send a list of waypoints as payload")
    parser.add_argument("--interval", type=float, default=2.0, help="Interval between sends in seconds (default: 2.0)")
    return parser.parse_args()


def main():
    args = parse_args()
    client = RedisCommClient(
        host=args.host,
        port=args.port,
        password=args.password,
        default_stream=args.stream,
    )

    print(f"[*] Redis 서버 연결 시도 중 ({args.host}:{args.port})...")
    if not client.connect():
        print("[!] Redis 서버에 연결할 수 없습니다.")
        sys.exit(1)

    print(f"[*] 스트림 '{args.stream}'으로 데이터 전송 시작...")

    count = 1
    try:
        while True:
            if args.send_list:
                # Example: Sending a list of waypoints
                data = {
                    "robot_id": args.stream,
                    "type": "waypoint_list",
                    "waypoints": [
                        {"seq": 1, "x": 10.0 + count, "y": 20.0, "status": "pending"},
                        {"seq": 2, "x": 15.0 + count, "y": 25.0, "status": "pending"},
                        {"seq": 3, "x": 20.0 + count, "y": 30.0, "status": "pending"},
                    ],
                    "timestamp": time.time(),
                }
            else:
                # Example: Sending robot telemetry
                data = {
                    "robot_id": args.stream,
                    "status": "patrol",
                    "battery": max(10, 100 - (count % 90)),
                    "pose": {"x": round(1.23 * count, 2), "y": 4.56, "yaw": 0.78},
                    "timestamp": time.time(),
                }

            msg_id = client.send_stream_data(args.stream, data)
            if msg_id:
                print(f"[Producer #{count}] 전송 성공! Stream={args.stream}, ID={msg_id}")
            else:
                print(f"[Producer #{count}] 전송 실패!")

            if args.once:
                break

            count += 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n전송 중단.")
    finally:
        client.close()


if __name__ == "__main__":
    main()

