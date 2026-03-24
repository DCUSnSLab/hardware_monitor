import rclpy
import asyncio
import websockets
import json
import threading

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, JointState, Imu, PointCloud2, CameraInfo, CompressedImage, Temperature
from hunter_msgs.msg import HunterStatus
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
from ublox_msgs.msg import NavPVT

TYPE_MAP = {
    "sensor_msgs/msg/JointState": JointState,
    "sensor_msgs/msg/NavSatFix": NavSatFix,
    "sensor_msgs/msg/Imu": Imu,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/CompressedImage": CompressedImage,
    "sensor_msgs/msg/Temperature": Temperature,

    "hunter_msgs/msg/HunterStatus": HunterStatus,
    
    "nav_msgs/msg/Odometry": Odometry,

    "tf2_msgs/msg/TFMessage": TFMessage,
    
    "geometry_msgs/msg/TwistWithCovarianceStamped": TwistWithCovarianceStamped,
    "geometry_msgs/msg/PoseStamped": PoseStamped,

    "ublox_msgs/msg/NavPVT": NavPVT
}


class RelayBridgeNode(Node):
    def __init__(self):
        super().__init__("relay_bridge_node")

        # 차량 파라미터 설정
        self.declare_parameter("vehicle_id", "scv")
        self.declare_parameter("relay_server_url", "ws://localhost:8080")

        self.vehicle_id = self.get_parameter("vehicle_id").get_parameter_value().string_value
        self.server_url = self.get_parameter("relay_server_url").get_parameter_value().string_value

        self.get_logger().info(f"vehicle ID: {self.vehicle_id}")
        self.get_logger().info(f"Relay Server: {self.server_url}")

        # self.subscribed_topics = {}

        self.create_subscription(
            NavSatFix,
            "/ublox_gps_node/fix",
            self.gps_callback,
            10
        )

        self.loop = asyncio.new_event_loop()

        self.thread = threading.Thread(target=self.start_loop)
        self.thread.start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect())

    def gps_callback(self, msg):
        self.get_logger().info(f"GPS: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

        if not hasattr(self, "ws") or self.ws is None:
            return
        
        data = {
            "type": "sensor_data",
            "topic": "/ublox_gps_node/fix",
            "data": {
                "lat": msg.latitude,
                "lon": msg.longitude,
                "alt": msg.altitude
            }
        }

        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(data)),
            self.loop
        )

    def topic_subscription(self, topic, msg_type):
        if topic in self.subscribed_topics:
            self.get_logger().info(f"⚠️ Already subscribed: {topic}")
            return

        self.subscribed_topics.add(topic)
        
        topic_type = TYPE_MAP.get(msg_type)

        if not topic_type:
            self.get_logger().warn(f"❌ Unknown type: {msg_type}")
            return
        
        self.get_logger().info(f"🔥 Subscribing to {topic}")

        self.create_subscription(
            topic_type,
            topic,
            lambda msg: self.topic_callback(topic, msg), 10
        )

        self.subscribed_topics[topic] = sub

    def topic_unsubscription(self, topic):
        if topic not in self.subscribed_topics:
            return
        
        sub = self.subscribed_topics.pop(topic)
        self.destroy_subscription(sub)

        self.get_logger().info(f"❌ Unsubscribed: {topic}")

    def topic_callback(self, topic, msg):
        if not hasattr(self, "ws") or self.ws is None:
            return

        data = {
            "type": "sensor_data",
            "topic": topic,
            "data": str(msg)
        }

        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(data)),
            self.loop
        )

    async def connect(self):
        while True:
            try:
                self.get_logger().info("Connecting to Relay Server...")

                async with websockets.connect(self.server_url) as ws:
                    self.ws = ws

                    self.get_logger().info("Connected to Relay Server")

                    await self.register_vehicle(ws)
                    await self.send_topic_list()

                    async for message in ws:
                        data = json.loads(message)

                        # if data["type"] == "subscribe_topic":
                        #     topic = data["topic"]
                        #     msg_type = data["msg_type"]
                            
                        #     self.topic_subscription(topic, msg_type)

                        # elif data["type"] == "unsubscribe_topic":
                        #     topic = data["topic"]
                        #     self.topic_unsubscription(topic)

            except Exception as e:
                self.ws = None
                self.get_logger().error(f"Connection error: {e}")
                self.get_logger().info("Retry in 3 seconds...")

                await asyncio.sleep(3)
    
    async def register_vehicle(self, ws):
        msg = {
            "type": "register",
            "role": "vehicle",
            "vehicle_id": self.vehicle_id
        }

        await ws.send(json.dumps(msg))

        self.get_logger().info("Vehicle registered")

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

        
def main(args=None):
    rclpy.init(args=args)
    node = RelayBridgeNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == "__main__":
    main()