import rclpy
import asyncio
import websockets
import json
import threading
import queue
import time
import struct
import numpy as np

from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

from sensor_msgs.msg import NavSatFix, JointState, Imu, PointCloud2, CameraInfo, CompressedImage, Temperature
from hunter_msgs.msg import HunterStatus
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped
from ublox_msgs.msg import NavPVT
from hardware_monitor2_interfaces.srv import Logging

GPS_TOPIC = "/ublox_gps_node/fix"


class RelayBridgeNode(Node):
    def __init__(self):
        super().__init__("relay_bridge_node")

        # 차량 파라미터 설정
        self.declare_parameter("vehicle_id", "default")
        self.declare_parameter("relay_server_url", "ws://localhost:8080")
        # bag 재생 데이터를 중계하는 경우 True로 실행 → 대시보드에서 'bag'으로 표시됨
        self.declare_parameter("is_bag", False)

        self.vehicle_id = self.get_parameter("vehicle_id").get_parameter_value().string_value
        self.server_url = self.get_parameter("relay_server_url").get_parameter_value().string_value
        self.is_bag = self.get_parameter("is_bag").get_parameter_value().bool_value

        self.get_logger().info(f"vehicle ID: {self.vehicle_id}")
        self.get_logger().info(f"Relay Server: {self.server_url}")

        self.subscribed_topics = {}
        self._last_topics = None
        self.logging_client = self.create_client(Logging, "/logging")
        self.logging_request_queue = queue.Queue()
        self.logging_request_timer = self.create_timer(0.05, self.process_logging_requests)

        # 토픽 목록 변경 감지: 주기적으로 확인해 바뀌면 relay로 재전송
        self.topic_check_timer = self.create_timer(3.0, self.check_topic_changes)
        self.gps_subscription = None
        self.sync_gps_subscription()

        self.loop = asyncio.new_event_loop()

        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.connect_task = self.loop.create_task(self.connect())
        try:
            self.loop.run_until_complete(self.connect_task)
        except asyncio.CancelledError:
            pass
        finally:
            self.loop.close()

    def stop_loop(self):
        task = getattr(self, "connect_task", None)
        if task is not None and not task.done() and self.loop.is_running():
            self.loop.call_soon_threadsafe(task.cancel)
        self.thread.join(timeout=5.0)

    def sync_gps_subscription(self):
        """실제 GPS publisher가 존재하는 동안에만 GPS를 구독한다."""
        has_publisher = bool(self.get_publishers_info_by_topic(GPS_TOPIC))

        if has_publisher and self.gps_subscription is None:
            self.gps_subscription = self.create_subscription(
                NavSatFix,
                GPS_TOPIC,
                self.gps_callback,
                10,
            )
            self.get_logger().info(f"GPS publisher detected: subscribed to {GPS_TOPIC}")
        elif not has_publisher and self.gps_subscription is not None:
            self.destroy_subscription(self.gps_subscription)
            self.gps_subscription = None
            self.get_logger().info(
                f"GPS publisher disappeared: unsubscribed from {GPS_TOPIC}"
            )

    # GPS publisher가 실제로 있을 때만 호출되는 callback
    def gps_callback(self, msg):
        if not hasattr(self, "ws") or self.ws is None:
            return

        data = {
            "type": "sensor_data",
            "topic": GPS_TOPIC,
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

                        elif data["type"] == "get_topic_list":
                            await self.send_topic_list()

                        elif data["type"] == "logging_request":
                            self.handle_logging_request(data)

            except Exception as e:
                self.ws = None
                self.get_logger().error(f"Connection error: {e}")

                await asyncio.sleep(1)

    # 차량 등록
    async def register_vehicle(self, ws):
        msg = {
            "type": "register",
            "role": "vehicle",
            "vehicle_id": self.vehicle_id,
            "is_bag": self.is_bag
        }

        await ws.send(json.dumps(msg))

        self.get_logger().info("Vehicle registered with relay server")

    def handle_logging_request(self, data):
        request_id = data.get("request_id")
        command = data.get("is_logging")
        topics = data.get("topics", [])
        bag_name = data.get("bag_name", "")

        if not request_id:
            self.get_logger().warning("logging_request missing request_id")
            return

        if command not in ("LoggingStart", "LoggingStop"):
            self.send_logging_response(
                request_id, success=False, error=f"Invalid logging command: {command}"
            )
            return

        if not isinstance(topics, list) or not all(isinstance(topic, str) for topic in topics):
            self.send_logging_response(
                request_id, success=False, error="topics must be a string array"
            )
            return

        if not isinstance(bag_name, str):
            self.send_logging_response(
                request_id, success=False, error="bag_name must be a string"
            )
            return

        self.get_logger().info(
            f"logging request queued: request_id={request_id} "
            f"command={command} topics={len(topics)} bag_name={bag_name!r}"
        )
        self.logging_request_queue.put({
            "request_id": request_id,
            "command": command,
            "topics": topics,
            "bag_name": bag_name,
            "deadline": time.monotonic() + 5.0,
        })

    def process_logging_requests(self):
        try:
            data = self.logging_request_queue.get_nowait()
        except queue.Empty:
            return

        request_id = data["request_id"]
        if not self.logging_client.service_is_ready():
            if time.monotonic() < data["deadline"]:
                self.logging_request_queue.put(data)
                return

            self.get_logger().error(
                f"/logging service unavailable: request_id={request_id}"
            )
            self.send_logging_response(
                request_id, success=False, error="/logging service is unavailable"
            )
            return

        request = Logging.Request()
        request.is_logging = data["command"]
        request.topics = data["topics"]
        request.bag_name = data["bag_name"]

        future = self.logging_client.call_async(request)
        future.add_done_callback(
            lambda completed, rid=request_id: self.logging_done_callback(rid, completed)
        )
        self.get_logger().info(
            f"/logging service called: request_id={request_id} command={data['command']}"
        )

    def logging_done_callback(self, request_id, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"/logging response: request_id={request_id} success={response.success} "
                f"is_logging={response.is_logging} status={response.logging_status} "
                f"bag_path={response.bag_path}"
            )
            self.send_logging_response(
                request_id,
                success=response.success,
                logging_status=response.logging_status,
                is_logging=response.is_logging,
                bag_path=response.bag_path,
                message=response.message,
                error="" if response.success else response.message,
            )
        except Exception as exc:
            self.get_logger().error(f"/logging service call failed: {exc}")
            self.send_logging_response(request_id, success=False, error=str(exc))

    def send_logging_response(
        self,
        request_id,
        success,
        logging_status="",
        is_logging=False,
        bag_path="",
        message="",
        error="",
    ):
        if not hasattr(self, "ws") or self.ws is None:
            return

        payload = {
            "type": "logging_response",
            "request_id": request_id,
            "success": success,
            "logging_status": logging_status,
            "is_logging": is_logging,
            "bag_path": bag_path,
            "message": message,
            "error": error,
        }
        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(payload)),
            self.loop,
        )

    # 차량의 모든 토픽 목록 + 타입 가져오기
    async def send_topic_list(self):
      topics = self.get_topic_names_and_types()

      topics_info = sorted(
          [{"name": t[0], "type": t[1][0]} for t in topics],
          key=lambda x: x["name"]
      )
      self._last_topics = topics_info

      msg = {
          "type" : "topic_list",
          "topics" : topics_info
      }

      await self.ws.send(json.dumps(msg))

    # 토픽 목록 변경 감지 후 relay로 재전송
    def check_topic_changes(self):
        self.sync_gps_subscription()

        if not hasattr(self, "ws") or self.ws is None:
            return

        topics = self.get_topic_names_and_types()
        topics_info = sorted(
            [{"name": t[0], "type": t[1][0]} for t in topics],
            key=lambda x: x["name"]
        )

        if topics_info == self._last_topics:
            return

        self._last_topics = topics_info
        msg = {"type": "topic_list", "topics": topics_info}
        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(msg)),
            self.loop
        )
        self.get_logger().info("📡 topic_list changed → resent")

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

        if topic == GPS_TOPIC:
            if self.gps_subscription is None:
                self.get_logger().warning(
                    f"GPS subscription skipped because {GPS_TOPIC} has no publisher"
                )
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

        # 대용량 토픽은 바이너리 프레임으로, 그 외는 기존 JSON으로 전송
        try:
            if isinstance(msg, PointCloud2):
                frame = self._encode_pointcloud(topic, msg)
                asyncio.run_coroutine_threadsafe(self.ws.send(frame), self.loop)
                return

            if isinstance(msg, CompressedImage):
                frame = self._encode_compressed_image(topic, msg)
                asyncio.run_coroutine_threadsafe(self.ws.send(frame), self.loop)
                return
        except Exception as e:
            self.get_logger().warn(f"binary encode failed for {topic}, fallback to JSON: {e}")

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

    # ---- 바이너리 프레임 인코딩 ----
    #
    # 프레임 레이아웃: [uint16 BE 헤더길이 H][H바이트 UTF-8 JSON 헤더][바이너리 페이로드]
    @staticmethod
    def _build_binary_frame(header, payload):
        header_bytes = json.dumps(header).encode("utf-8")
        return struct.pack(">H", len(header_bytes)) + header_bytes + payload

    def _pointcloud_to_xyz(self, msg):
        field_names = [f.name for f in msg.fields]
        names = ("x", "y", "z", "intensity") if "intensity" in field_names else ("x", "y", "z")

        try:
            arr = point_cloud2.read_points_numpy(msg, field_names=names, skip_nans=True)
            arr = np.asarray(arr, dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, len(names))
        except Exception:
            # 구버전 sensor_msgs_py 폴백(느림)
            pts = point_cloud2.read_points(msg, field_names=names, skip_nans=True)
            arr = np.array([tuple(p) for p in pts], dtype=np.float32)
            if arr.size == 0:
                arr = arr.reshape(-1, len(names))

        return arr, list(names)

    def _encode_pointcloud(self, topic, msg):
        arr, names = self._pointcloud_to_xyz(msg)

        payload = np.ascontiguousarray(arr, dtype="<f4").tobytes()

        header = {
            "type": "sensor_data",
            "vehicle_id": self.vehicle_id,
            "topic": topic,
            "msg_type": "sensor_msgs/msg/PointCloud2",
            "fields": names,
            "count": int(arr.shape[0]),
            "sent_at": time.time() * 1000,
        }
        return self._build_binary_frame(header, payload)

    def _encode_compressed_image(self, topic, msg):
        payload = bytes(msg.data)  # 이미 압축된 jpeg/png 바이트

        header = {
            "type": "sensor_data",
            "vehicle_id": self.vehicle_id,
            "topic": topic,
            "msg_type": "sensor_msgs/msg/CompressedImage",
            "format": msg.format,
            "sent_at": time.time() * 1000,
        }
        return self._build_binary_frame(header, payload)

    # 토픽 구독 해제
    def topic_unsubscription(self, topic):
        if topic == GPS_TOPIC:
            self.get_logger().info(
                f"Persistent GPS unsubscribe ignored: {GPS_TOPIC}"
            )
            return

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
        node.stop_loop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()