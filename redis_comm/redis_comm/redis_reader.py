import argparse
import json
import sys
import time

try:
    from redis_comm.redis_client import RedisCommClient
except ImportError:
    from .redis_client import RedisCommClient


def parse_args():
    parser = argparse.ArgumentParser(
        description="Redis Stream & Key reader tool for ROS 2 robot system"
    )
    parser.add_argument("--host", default="172.16.0.200", help="Redis server IP address (default: 172.16.0.200)")
    parser.add_argument("--port", type=int, default=6379, help="Redis server port (default: 6379)")
    parser.add_argument("--password", default="1qaz", help="Redis password (default: 1qaz)")
    parser.add_argument("--stream", default="SP-OS-RW-R1", help="Target stream key name (default: SP-OS-RW-R1)")
    parser.add_argument("--count", type=int, default=10, help="Number of messages to retrieve (default: 10)")
    parser.add_argument("--follow", action="store_true", help="Continuously poll for new messages")
    parser.add_argument("--interval", type=float, default=2.0, help="Poll interval in seconds (default: 2.0)")
    return parser.parse_args()


def print_banner(host: str, port: int, stream: str):
    print("=" * 60)
    print("   [Redis Comm] Stream & Key Reader")
    print(f"   - Target Server : {host}:{port}")
    print(f"   - Target Stream : {stream}")
    print("=" * 60)


def display_keys_and_stream(client: RedisCommClient, stream_name: str, count: int):
    # 1. List all keys
    print("\n[1] Redis에 등록된 Key 목록 조회:")
    keys = client.get_all_keys("*")
    if not keys:
        print("   -> 등록된 키가 없거나 조회 실패했습니다.")
    else:
        for idx, k in enumerate(keys, 1):
            k_type = client.get_key_type(k)
            marker = " (<- 대상 스트림)" if k == stream_name else ""
            print(f"   {idx:2d}. {k} [Type: {k_type}]{marker}")

    # 2. Read stream messages
    stream_type = client.get_key_type(stream_name)
    total_len = client.get_stream_length(stream_name)
    print(f"\n[2] '{stream_name}' 스트림 메시지 리스트 (총 {total_len}개 중 최근 {count}개):")

    if stream_type == "none" and total_len == 0:
        print(f"   -> 스트림 '{stream_name}'에 아직 데이터가 없거나 키가 생성되지 않았습니다.")
        return

    messages = client.read_stream_list(stream_name=stream_name, count=count, reverse=True)
    if not messages:
        print("   -> 메시지가 비어 있습니다.")
        return

    for idx, msg in enumerate(messages, 1):
        msg_id = msg["id"]
        parsed = msg.get("parsed_payload")
        print(f"   [{idx:2d}] ID: {msg_id}")
        if isinstance(parsed, (dict, list)):
            pretty = json.dumps(parsed, indent=6, ensure_ascii=False)
            print(f"        Payload:\n{pretty}")
        else:
            print(f"        Fields : {msg['fields']}")
    print("-" * 60)


def main():
    args = parse_args()
    print_banner(args.host, args.port, args.stream)

    client = RedisCommClient(
        host=args.host,
        port=args.port,
        password=args.password,
        default_stream=args.stream,
    )

    print(f"[*] Redis 서버 연결 시도 중 ({args.host}:{args.port})...")
    if not client.connect():
        print("[!] Redis 서버에 연결할 수 없습니다.")
        print("    - 사무실 네트워크(172.16.0.200)에 연결되어 있는지 확인해주세요.")
        print("    - 로컬 테스트 시에는 --host localhost 로 실행할 수 있습니다.")
        sys.exit(1)

    display_keys_and_stream(client, args.stream, args.count)

    if args.follow:
        print("\n[*] 실시간 모니터링 시작 (Ctrl+C 로 종료)...")
        last_seen_id = None
        try:
            while True:
                messages = client.read_stream_list(stream_name=args.stream, count=1, reverse=True)
                if messages:
                    latest = messages[0]
                    if latest["id"] != last_seen_id:
                        last_seen_id = latest["id"]
                        print(f"\n[신규 수신] ID: {latest['id']}")
                        payload = latest.get("parsed_payload")
                        if payload is not None:
                            print(f"Payload: {payload}")
                        else:
                            print(f"Fields: {latest['fields']}")
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n모니터링 종료.")
        finally:
            client.close()
    else:
        client.close()


if __name__ == "__main__":
    main()

