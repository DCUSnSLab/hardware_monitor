import rclpy
import asyncio
import websockets
import json
import os
import subprocess
import signal
import threading
import queue
import time
import struct
import numpy as np

from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

from sensor_msgs.msg import NavSatFix, PointCloud2, CompressedImage
from hardware_monitor2_interfaces.srv import Logging

GPS_TOPIC = "/ublox_gps_node/fix"

# 로깅으로 저장되는 bag 폴더 위치(대시보드의 bag 목록 조회/재생 대상)
BAG_DIR = os.path.expanduser("~/hardware_monitor/bag")


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

        # ---- bag 재생 상태 ----
        # 재생은 ros2 bag play 서브프로세스로 처리한다(배포판 서비스에 의존하지 않는 respawn 모델).
        #  - open   = bag 로드(일시정지 상태로 대기, 재생 X) + 토픽 목록을 metadata에서 고정
        #  - play   = 보존 위치에서 --start-offset 으로 시작/재개
        #  - pause  = 프로세스 종료 + 현재 위치 보존
        #  - seek   = 원하는 위치에서 재시작(일시정지 중이면 위치만 갱신)
        #  - rate   = 현재 위치에서 --rate 로 재시작
        #  - stop   = 종료 + 상태 초기화
        # 현재 위치는 rosbag2가 직접 제공하지 않으므로, 시작시각+배속으로 추정한다.
        self.bag_proc = None
        self.bag_path = ""
        self.bag_name = ""
        self.bag_duration = 0.0        # 초
        self.bag_rate = 1.0
        self.bag_paused = False
        self.bag_base_position = 0.0   # 마지막 (재)시작/일시정지 시점의 위치(초)
        self.bag_play_started = None   # 재생(비일시정지) 구간 시작 monotonic
        self.bag_loaded = False        # bag이 로드된 세션인지(로드 중엔 토픽 목록 고정)
        self.bag_topics = None         # 로드된 bag의 토픽 목록(metadata 기준, 고정)
        self.bag_status_timer = self.create_timer(0.25, self.publish_bag_status)

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
        self._kill_bag_proc()
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
                                continue

                            self.topic_unsubscription(topic)

                        elif data["type"] == "get_topic_list":
                            await self.send_topic_list()

                        elif data["type"] == "logging_request":
                            self.handle_logging_request(data)

                        # bag 파일 목록 요청 → 실제 BAG_DIR 조회 후 응답
                        elif data["type"] == "bag_list_request":
                            await self.send_bag_list(
                                data.get("request_id"), data.get("reply_to")
                            )

                        # bag 재생 제어(open/stop/play/pause/seek/rate)
                        elif data["type"] == "bag_playback_request":
                            await self.handle_bag_playback_request(data)

            except Exception as e:
                self.ws = None
                self.get_logger().error(f"Connection error: {e}")

                # 연결이 끊기면(볼 사람이 없음) 재생 중이던 bag을 종료한다.
                if self.bag_loaded or self.bag_proc is not None:
                    self._reset_bag_session()
                    self.get_logger().info("Connection lost → bag playback terminated")

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

    # bag 파일 목록 조회 → bag_list_response 로 응답
    async def send_bag_list(self, request_id, reply_to):
        bags = []
        error = ""

        try:
            entries = sorted(os.listdir(BAG_DIR))
        except FileNotFoundError:
            entries = []            # 아직 bag 폴더가 없으면 빈 목록
        except Exception as exc:
            entries = []
            error = f"failed to list bag dir: {exc}"

        for name in entries:
            path = os.path.join(BAG_DIR, name)
            meta_path = os.path.join(path, "metadata.yaml")
            # ros2 bag은 metadata.yaml을 포함한 폴더 형태만 유효
            if not os.path.isdir(path) or not os.path.isfile(meta_path):
                continue

            info = {"name": name, "path": path, "mtime": os.path.getmtime(path)}
            dur = self._read_bag_duration(path)
            if dur > 0:
                info["duration_sec"] = dur
            bags.append(info)

        payload = {
            "type": "bag_list_response",
            "request_id": request_id,
            "reply_to": reply_to,          # 릴레이가 요청 user로 되돌리는 데 사용
            "vehicle_id": self.vehicle_id,
            "bags": bags,
            "success": not error,
            "error": error,
        }

        if hasattr(self, "ws") and self.ws is not None:
            await self.ws.send(json.dumps(payload))

    # ---- bag 재생 ----
    def _load_metadata(self, bag_path):
        meta_path = os.path.join(bag_path, "metadata.yaml")
        try:
            import yaml
            with open(meta_path, "r") as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}

    def _read_bag_duration(self, bag_path):
        binfo = self._load_metadata(bag_path).get("rosbag2_bagfile_information", {}) or {}
        dur = (binfo.get("duration", {}) or {}).get("nanoseconds")
        if isinstance(dur, (int, float)):
            return dur / 1e9
        return 0.0

    def _read_bag_topics(self, bag_path):
        """metadata.yaml에서 bag이 담은 토픽 목록을 읽는다(재생 전에도 사이드바에 고정 표시)."""
        binfo = self._load_metadata(bag_path).get("rosbag2_bagfile_information", {}) or {}
        topics = []
        for entry in (binfo.get("topics_with_message_count") or []):
            tm = (entry.get("topic_metadata") or {})
            name = tm.get("name")
            ttype = tm.get("type")
            if name and ttype:
                topics.append({"name": name, "type": ttype})
        return sorted(topics, key=lambda x: x["name"])

    def _bag_current_position(self):
        pos = self.bag_base_position
        if self.bag_play_started is not None and not self.bag_paused:
            pos += (time.monotonic() - self.bag_play_started) * self.bag_rate
        if self.bag_duration > 0:
            pos = max(0.0, min(pos, self.bag_duration))
        else:
            pos = max(0.0, pos)
        return pos

    def _bag_state(self):
        if not self.bag_path:
            return "idle"
        if self.bag_paused:
            return "paused"
        if self.bag_proc is not None and self.bag_proc.poll() is None:
            return "playing"
        return "idle"

    def _kill_bag_proc(self):
        proc = self.bag_proc
        self.bag_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=1.0)
                return
            except subprocess.TimeoutExpired:
                pass
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as exc:
            self.get_logger().warning(f"bag play kill failed: {exc}")

    def _reset_bag_session(self):
        """재생 종료 + 상태 초기화(연결 끊김/stop 공통)."""
        self._kill_bag_proc()
        self.bag_paused = False
        self.bag_base_position = 0.0
        self.bag_play_started = None
        self.bag_path = ""
        self.bag_name = ""
        self.bag_duration = 0.0
        self.bag_loaded = False
        self.bag_topics = None

    def _spawn_bag_proc(self, offset):
        """현재 bag_path/bag_rate로 offset(초)부터 ros2 bag play 재시작."""
        self._kill_bag_proc()
        if not self.bag_path:
            return False, "no bag opened"

        offset = max(0.0, float(offset))
        if self.bag_duration > 0:
            offset = min(offset, self.bag_duration)

        cmd = [
            "ros2", "bag", "play", self.bag_path,
            "--rate", str(self.bag_rate),
            "--start-offset", str(offset),
        ]
        try:
            self.bag_proc = subprocess.Popen(cmd, start_new_session=True)
            self.get_logger().info(
                f"▶️ ros2 bag play: {self.bag_path} rate={self.bag_rate} offset={offset:.2f}"
            )
            return True, ""
        except Exception as exc:
            self.bag_proc = None
            return False, f"failed to start ros2 bag play: {exc}"

    async def handle_bag_playback_request(self, data):
        request_id = data.get("request_id")
        reply_to = data.get("reply_to")
        action = data.get("action")
        success = True
        error = ""

        if action == "open":
            bag_path = data.get("bag_path") or ""
            if not bag_path:
                success, error = False, "bag_path is required for open"
            else:
                # 로드만 하고 재생은 하지 않는다(일시정지 대기 → 사용자가 play를 눌러야 재생).
                self._kill_bag_proc()
                self.bag_path = bag_path
                self.bag_name = os.path.basename(bag_path.rstrip("/"))
                self.bag_duration = self._read_bag_duration(bag_path)
                self.bag_topics = self._read_bag_topics(bag_path)
                self.bag_loaded = True
                self.bag_rate = float(data.get("rate") or 1.0)
                self.bag_base_position = 0.0
                self.bag_play_started = None
                self.bag_paused = True
                # 사이드바에 bag 토픽 목록을 고정 표시(정지해도 안 바뀜)
                await self.send_topic_list()

        elif action == "stop":
            self._reset_bag_session()
            # 라이브 토픽 목록으로 복귀
            await self.send_topic_list()

        elif action == "pause":
            if self.bag_path and not self.bag_paused:
                self.bag_base_position = self._bag_current_position()
                self._kill_bag_proc()
                self.bag_paused = True
                self.bag_play_started = None

        elif action in ("play", "resume"):
            if not self.bag_path:
                success, error = False, "no bag opened"
            else:
                if data.get("rate"):
                    self.bag_rate = float(data.get("rate"))
                self.bag_paused = False
                ok, err = self._spawn_bag_proc(self.bag_base_position)
                if ok:
                    self.bag_play_started = time.monotonic()
                else:
                    success, error = False, err

        elif action == "seek":
            if not self.bag_path:
                success, error = False, "no bag opened"
            else:
                pos = float(
                    data.get("position_seconds")
                    if data.get("position_seconds") is not None
                    else (data.get("position") or 0.0)
                )
                if self.bag_duration > 0:
                    pos = max(0.0, min(pos, self.bag_duration))
                self.bag_base_position = pos
                if not self.bag_paused:
                    ok, err = self._spawn_bag_proc(pos)
                    if ok:
                        self.bag_play_started = time.monotonic()
                    else:
                        success, error = False, err

        elif action == "rate":
            if not self.bag_path:
                success, error = False, "no bag opened"
            else:
                self.bag_base_position = self._bag_current_position()
                self.bag_rate = float(data.get("rate") or self.bag_rate)
                if not self.bag_paused:
                    ok, err = self._spawn_bag_proc(self.bag_base_position)
                    if ok:
                        self.bag_play_started = time.monotonic()
                    else:
                        success, error = False, err

        else:
            success, error = False, f"unknown playback action: {action}"

        state = self._bag_state()
        payload = {
            "type": "bag_playback_response",
            "request_id": request_id,
            "reply_to": reply_to,
            "vehicle_id": self.vehicle_id,
            "success": success,
            "error": error,
            "state": state,
            "bag_path": self.bag_path,
            "bag_name": self.bag_name,
            "current_time": self._bag_current_position(),
            "duration": self.bag_duration,
            "rate": self.bag_rate,
            "is_playing": state == "playing",
        }
        if hasattr(self, "ws") and self.ws is not None:
            await self.ws.send(json.dumps(payload))

    # 주기적으로 재생 상태를 relay(→ 모든 user)로 전송
    def publish_bag_status(self):
        if not hasattr(self, "ws") or self.ws is None:
            return
        if not self.bag_path:
            return  # 열린 bag 없으면 상태 미전송

        # 재생 중이었는데 프로세스가 끝났으면(=bag 끝까지 재생) 완료 처리(끝 위치에서 정지)
        if (not self.bag_paused and self.bag_proc is not None
                and self.bag_proc.poll() is not None):
            self.bag_base_position = self.bag_duration
            self.bag_play_started = None
            self.bag_proc = None
            self.bag_paused = True

        pos = self._bag_current_position()
        state = self._bag_state()
        payload = {
            "type": "bag_playback_status",
            "vehicle_id": self.vehicle_id,
            "bag_path": self.bag_path,
            "bag_name": self.bag_name,
            "state": state,
            "current_time": pos,
            "duration": self.bag_duration,
            "rate": self.bag_rate,
            "is_playing": state == "playing",
        }
        asyncio.run_coroutine_threadsafe(
            self.ws.send(json.dumps(payload)),
            self.loop
        )

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
    # bag이 로드된 동안에는 bag의 토픽 목록(metadata 기준)을 고정으로 보낸다(정지해도 안 바뀜).
    async def send_topic_list(self):
        if self.bag_loaded and self.bag_topics is not None:
            topics_info = self.bag_topics
        else:
            topics = self.get_topic_names_and_types()
            topics_info = sorted(
                [{"name": t[0], "type": t[1][0]} for t in topics],
                key=lambda x: x["name"]
            )
        self._last_topics = topics_info

        msg = {
            "type": "topic_list",
            "topics": topics_info
        }
        if hasattr(self, "ws") and self.ws is not None:
            await self.ws.send(json.dumps(msg))

    # 토픽 목록 변경 감지 후 relay로 재전송
    def check_topic_changes(self):
        self.sync_gps_subscription()

        if not hasattr(self, "ws") or self.ws is None:
            return

        # bag 로드 중에는 토픽 목록을 고정한다(재생 정지로 토픽이 사라져도 사이드바 유지).
        if self.bag_loaded:
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

    def destroy_node(self):
        # 종료 시 재생 중인 bag 프로세스 정리
        self._kill_bag_proc()
        return super().destroy_node()


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
