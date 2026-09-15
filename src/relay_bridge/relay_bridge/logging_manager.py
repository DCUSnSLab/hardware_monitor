"""rosbag 로깅 담당.

/logging 서비스 클라이언트 + 요청 큐 + 응답 생성을 한 곳에 모은다.
WS·타이머는 노드가 소유하고, 이 매니저는:
 - 서비스 클라이언트(logging_client)와 로거, send 콜백(send_fn)을 주입받고
 - handle_request(data) 로 검증·큐잉, process() 로 서비스 호출을 처리하며
 - 응답은 send_fn(payload_dict) 으로 내보낸다(실제 전송은 노드가 담당).
"""
import queue
import time

from hardware_monitor2_interfaces.srv import Logging


class LoggingManager:
    def __init__(self, logging_client, logger, send_fn):
        self.client = logging_client
        self.logger = logger
        self.send_fn = send_fn          # send_fn(payload_dict) → logging_response 전송
        self.request_queue = queue.Queue()

    def _respond(self, request_id, success, logging_status="", is_logging=False,
                 bag_path="", message="", error=""):
        self.send_fn({
            "type": "logging_response",
            "request_id": request_id,
            "success": success,
            "logging_status": logging_status,
            "is_logging": is_logging,
            "bag_path": bag_path,
            "message": message,
            "error": error,
        })

    def handle_request(self, data):
        request_id = data.get("request_id")
        command = data.get("is_logging")
        topics = data.get("topics", [])
        bag_name = data.get("bag_name", "")

        if not request_id:
            if self.logger:
                self.logger.warning("logging_request missing request_id")
            return

        if command not in ("LoggingStart", "LoggingStop"):
            self._respond(request_id, success=False, error=f"Invalid logging command: {command}")
            return

        if not isinstance(topics, list) or not all(isinstance(topic, str) for topic in topics):
            self._respond(request_id, success=False, error="topics must be a string array")
            return

        if not isinstance(bag_name, str):
            self._respond(request_id, success=False, error="bag_name must be a string")
            return

        if self.logger:
            self.logger.info(
                f"logging request queued: request_id={request_id} "
                f"command={command} topics={len(topics)} bag_name={bag_name!r}"
            )
        self.request_queue.put({
            "request_id": request_id,
            "command": command,
            "topics": topics,
            "bag_name": bag_name,
            "deadline": time.monotonic() + 5.0,
        })

    def process(self):
        try:
            data = self.request_queue.get_nowait()
        except queue.Empty:
            return

        request_id = data["request_id"]
        if not self.client.service_is_ready():
            if time.monotonic() < data["deadline"]:
                self.request_queue.put(data)
                return

            if self.logger:
                self.logger.error(f"/logging service unavailable: request_id={request_id}")
            self._respond(request_id, success=False, error="/logging service is unavailable")
            return

        request = Logging.Request()
        request.is_logging = data["command"]
        request.topics = data["topics"]
        request.bag_name = data["bag_name"]

        future = self.client.call_async(request)
        future.add_done_callback(
            lambda completed, rid=request_id: self._done(rid, completed)
        )
        if self.logger:
            self.logger.info(
                f"/logging service called: request_id={request_id} command={data['command']}"
            )

    def _done(self, request_id, future):
        try:
            response = future.result()
            if self.logger:
                self.logger.info(
                    f"/logging response: request_id={request_id} success={response.success} "
                    f"is_logging={response.is_logging} status={response.logging_status} "
                    f"bag_path={response.bag_path}"
                )
            self._respond(
                request_id,
                success=response.success,
                logging_status=response.logging_status,
                is_logging=response.is_logging,
                bag_path=response.bag_path,
                message=response.message,
                error="" if response.success else response.message,
            )
        except Exception as exc:
            if self.logger:
                self.logger.error(f"/logging service call failed: {exc}")
            self._respond(request_id, success=False, error=str(exc))
