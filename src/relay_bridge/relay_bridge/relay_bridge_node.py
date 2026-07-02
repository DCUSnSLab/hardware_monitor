import rclpy
import asyncio
import websockets
import json
import threading
import time

from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from urllib.parse import urlparse

from sensor_msgs.msg import NavSatFix, JointState, Imu, PointCloud2, CameraInfo, CompressedImage, Temperature
from hunter_msgs.msg import HunterStatus
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
from ublox_msgs.msg import NavPVT

class RelayBridgeNode(Node):
    def __init__(self):
        super().__init__("relay_bridge_node")

        # 차량 파라미터 설정
        self.declare_parameter("vehicle_id", "bag(test)")
        self.declare_parameter("relay_server_url", "ws://localhost:8080")

        self.vehicle_id = self.get_parameter("vehicle_id").get_parameter_value().string_value
        self.server_url = self.get_parameter("relay_server_url").get_parameter_value().string_value

        self.get_logger().info(f"vehicle ID: {self.vehicle_id}")
        self.get_logger().info(f"Relay Server: {self.server_url}")

        self.subscribed_topics = {}

        self.create_subscription(
            NavSatFix,
            "/ublox_gps_node/fix",
            self.gps_callback,
            10
        )

        self.loop = asyncio.new_event_loop()

        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect())
        
    # gps callback 함수 (gps 값은 무조건 받도록)
    def gps_callback(self, msg):
        if not hasattr(self, "ws") or self.ws is None:
            return
        
        data = {
            "type": "sensor_data",
            "topic": "/ublox_gps_node/fix",
            "data": {
                "lat": msg.latitude,
                "lon": msg.longitude,
                # "alt": msg.altitude
            }
        }

        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(data)),
            self.loop
        )

    async def connect(self):
        while rclpy.ok():
            try:
                self.get_logger().info("Connecting to Relay Server...")

                async with websockets.connect(self.server_url) as ws:
                    self.ws = ws

                    self.get_logger().info("Connected to Relay Server")

                    await self.register_vehicle(ws)
                    await self.send_topic_list()

                    async for message in ws:
                        data = json.loads(message)

                        self.get_logger().info(f"📦 received: {data}")

                        # 토픽 구독
                        if data["type"] == "subscribe_topic":
                            topic = data["topic"]
                            msg_type = data["msg_type"]

                            if not topic or not msg_type:
                                self.get_logger().warn(f"Invalid subscribe message: {data}")

                            self.topic_subscription(topic, msg_type)

                        # 토픽 구독 해제
                        elif data["type"] == "unsubscribe_topic":
                            topic = data["topic"]

                            if not topic:
                                self.get_logger().warn(f"Invalid unsubscribe message: {data}")
                                return
    
                            self.topic_unsubscription(topic)

            except Exception as e:
                self.ws = None
                self.get_logger().error(f"Connection error: {e}")

                await asyncio.sleep(1)

    # 차량 등록
    async def register_vehicle(self, ws):
        parsed = urlparse(self.server_url)
        host = parsed.hostname
        rosbridge_ip = f"ws://{host}:9090"

        msg = {
            "type": "register",
            "role": "vehicle",
            "vehicle_id": self.vehicle_id,
            "rosbridge_ip": rosbridge_ip
        }

        await ws.send(json.dumps(msg))

        self.get_logger().info(f"Vehicle registered with rosbridge: {rosbridge_ip}")

    # 차량의 모든 토픽 목록 + 타입 가져오기
    async def send_topic_list(self):
      topics = self.get_topic_names_and_types()

      topics_info = [
          {"name": t[0], "type": t[1][0]}
          for t in topics
      ]

      msg = {
          "type" : "topic_list",
          "topics" : topics_info
      }

      await self.ws.send(json.dumps(msg))

    # 토픽 구독
    def topic_subscription(self, topic, msg_type):
        if topic in self.subscribed_topics:
            self.get_logger().info(f"⚠️ Already subscribed: {topic}")
            return
        
        topic_type = get_message(msg_type)

        if not topic_type:
            self.get_logger().warn(f"❌ Unknown type: {msg_type}")
            return
        
        self.get_logger().info(f"🔥 Subscribing to {topic}")

        if topic == "/ublox_gps_node/fix":
            return

        sub = self.create_subscription(
            topic_type,
            topic,
            lambda msg: self.topic_callback(topic, msg), 10
        )

        self.subscribed_topics[topic] = sub

    def topic_callback(self, topic, msg):
        if not hasattr(self, "ws") or self.ws is None:
            return

        data = {
            "type": "sensor_data",
            "topic": topic,
            "data": message_to_ordereddict(msg),
            "sent_at": time.time() * 1000
        }

        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(data)),
            self.loop
        )

    # 토픽 구독 해제
    def topic_unsubscription(self, topic):
        if topic not in self.subscribed_topics:
            return
        
        sub = self.subscribed_topics.pop(topic)
        self.destroy_subscription(sub)

        self.get_logger().info(f"❌ Unsubscribed: {topic}")

        
def main(args=None):
    rclpy.init(args=args)
    node = RelayBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.loop.call_soon_threadsafe(node.loop.stop)
        node.thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()