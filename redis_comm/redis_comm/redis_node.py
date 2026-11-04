import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from redis_comm.redis_client import RedisCommClient


class RedisCommNode(Node):
    """ROS 2 Node for communicating with Redis server and publishing stream data."""

    def __init__(self):
        super().__init__('redis_comm_node')

        # Declare ROS 2 parameters
        self.declare_parameter('host', '172.16.0.200')
        self.declare_parameter('port', 6379)
        self.declare_parameter('password', '1qaz')
        self.declare_parameter('stream_name', 'SP-OS-RW-R1')
        self.declare_parameter('poll_rate_hz', 1.0)
        self.declare_parameter('max_fetch_count', 10)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.stream_name = self.get_parameter('stream_name').get_parameter_value().string_value
        self.poll_rate = self.get_parameter('poll_rate_hz').get_parameter_value().double_value
        self.fetch_count = self.get_parameter('max_fetch_count').get_parameter_value().integer_value

        self.get_logger().info(
            f"Initializing RedisCommNode for {self.host}:{self.port}, Stream: '{self.stream_name}'"
        )

        # Initialize Redis Client
        self.client = RedisCommClient(
            host=self.host,
            port=self.port,
            password=self.password,
            default_stream=self.stream_name,
            socket_timeout=2.0,
        )

        # ROS 2 Publisher
        self.publisher_ = self.create_publisher(String, '/redis/stream_data', 10)

        # Connection status tracking
        self.is_connected = False
        self.last_msg_id = None

        # Create timer
        period = 1.0 / max(0.1, self.poll_rate)
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        if not self.is_connected:
            self.get_logger().info(f"Connecting to Redis at {self.host}:{self.port}...")
            if self.client.connect():
                self.is_connected = True
                self.get_logger().info(f"Connected to Redis successfully!")
                keys = self.client.get_all_keys("*")
                self.get_logger().info(f"Available keys in Redis: {keys}")
            else:
                self.get_logger().warn(f"Waiting for Redis connection at {self.host}:{self.port}...")
                return

        try:
            messages = self.client.read_stream_list(
                stream_name=self.stream_name,
                count=self.fetch_count,
                reverse=True,
            )

            if messages:
                latest = messages[0]
                if latest["id"] != self.last_msg_id:
                    self.last_msg_id = latest["id"]
                    self.get_logger().info(
                        f"New message from '{self.stream_name}': ID={latest['id']}, "
                        f"Payload={latest.get('parsed_payload')}"
                    )

                # Publish latest message as JSON string to ROS 2 topic
                msg = String()
                msg.data = json.dumps(messages, ensure_ascii=False)
                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading from Redis: {e}")
            self.is_connected = False

    def destroy_node(self):
        if self.client:
            self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RedisCommNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

