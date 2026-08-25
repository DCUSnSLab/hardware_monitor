#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from hardware_monitor2_interfaces.srv import Logging


class RosbagLoggingServer(Node):
    def __init__(self):
        super().__init__("logging_server")
        self.declare_parameter("bag_dir", os.path.expanduser("~/hardware_monitor/bag"))
        value = self.get_parameter("bag_dir").get_parameter_value().string_value
        self.bag_dir = os.path.abspath(os.path.expanduser(value))
        self.srv = self.create_service(Logging, "/logging", self.handle_logging)
        self.bag_proc = None
        self.bag_log = None
        self.current_bag_path = ""
        self.current_log_path = ""
        self.last_bag_path = ""
        self.create_timer(1.0, self.monitor_recording)
        self.get_logger().info(
            f"logging_server ready: service=/logging bag_dir={self.bag_dir}"
        )

    @staticmethod
    def respond(response, success, is_logging, status, bag_path="", message=""):
        response.success = success
        response.is_logging = is_logging
        response.logging_status = status
        response.bag_path = bag_path
        response.message = message
        return response

    @staticmethod
    def clean_name(raw_name):
        name = (raw_name or "").strip()
        if not name:
            return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        name = re.sub(r"[^\w.-]+", "_", os.path.basename(name), flags=re.UNICODE)
        return name.strip("._") or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    def unique_path(self, name):
        path = os.path.join(self.bag_dir, name)
        if not os.path.exists(path):
            return path
        return f"{path}_{datetime.now().strftime('%H-%M-%S-%f')}"

    def close_log(self):
        if self.bag_log is None:
            return
        try:
            self.bag_log.flush()
            self.bag_log.close()
        except Exception:
            pass
        self.bag_log = None

    def log_tail(self):
        if self.bag_log is not None:
            try:
                self.bag_log.flush()
            except Exception:
                pass
        if not self.current_log_path or not os.path.isfile(self.current_log_path):
            return ""
        try:
            with open(self.current_log_path, "rb") as stream:
                stream.seek(0, os.SEEK_END)
                stream.seek(max(0, stream.tell() - 4096))
                return stream.read().decode("utf-8", errors="replace").strip()
        except Exception:
            return ""

    def clear_process(self):
        self.close_log()
        self.bag_proc = None
        self.current_bag_path = ""
        self.current_log_path = ""

    def wait_for_start(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            code = self.bag_proc.poll()
            if code is not None:
                detail = self.log_tail()
                error = f"ros2 bag exited during startup (code={code})"
                return False, error + (f": {detail}" if detail else "")
            if os.path.isdir(self.current_bag_path):
                return True, ""
            time.sleep(0.1)
        return False, "ros2 bag did not create its output directory within 5 seconds"

    def stop_process(self):
        if self.bag_proc is None or self.bag_proc.poll() is not None:
            return True, ""
        try:
            pgid = os.getpgid(self.bag_proc.pid)
        except Exception as exc:
            return False, f"failed to get process group: {exc}"

        for sig, timeout, label in (
            (signal.SIGINT, 10.0, "SIGINT"),
            (signal.SIGTERM, 3.0, "SIGTERM"),
            (signal.SIGKILL, 2.0, "SIGKILL"),
        ):
            try:
                os.killpg(pgid, sig)
                self.get_logger().info(
                    f"sent {label}: pgid={pgid} bag={self.current_bag_path}"
                )
            except ProcessLookupError:
                return True, ""
            except Exception as exc:
                return False, f"failed to send {label}: {exc}"
            try:
                self.bag_proc.wait(timeout=timeout)
                return True, ""
            except subprocess.TimeoutExpired:
                pass
        return self.bag_proc.poll() is not None, "ros2 bag did not terminate"

    def handle_logging(self, request, response):
        command = str(request.is_logging)
        topics = [topic.strip() for topic in (request.topics or []) if topic.strip()]
        self.get_logger().info(
            f"logging request received: command={command} "
            f"topics={len(topics)} bag_name={request.bag_name!r}"
        )

        if command == "LoggingStart":
            if self.bag_proc is not None and self.bag_proc.poll() is None:
                return self.respond(
                    response, True, True, "AlreadyRecording",
                    self.current_bag_path, "A recording is already active."
                )
            if self.bag_proc is not None:
                self.clear_process()
            try:
                os.makedirs(self.bag_dir, exist_ok=True)
            except Exception as exc:
                message = f"failed to create bag directory {self.bag_dir}: {exc}"
                self.get_logger().error(message)
                return self.respond(response, False, False, "StartFailed", message=message)

            requested_name = (request.bag_name or "").strip()
            name = self.clean_name(requested_name)
            if requested_name and requested_name != name:
                self.get_logger().warning(
                    f"bag name sanitized: {requested_name!r} -> {name!r}"
                )

            output_path = self.unique_path(name)
            log_path = f"{output_path}.rosbag.log"
            cmd = ["ros2", "bag", "record", "-o", output_path]
            cmd.extend(topics if topics else ["-a"])
            self.current_bag_path = output_path
            self.current_log_path = log_path
            self.last_bag_path = output_path

            try:
                self.bag_log = open(log_path, "a", encoding="utf-8", buffering=1)
                self.bag_proc = subprocess.Popen(
                    cmd,
                    stdout=self.bag_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception as exc:
                message = f"failed to start ros2 bag: {exc}"
                self.get_logger().error(message)
                self.clear_process()
                return self.respond(
                    response, False, False, "StartFailed", output_path, message
                )

            started, error = self.wait_for_start()
            if not started:
                self.get_logger().error(
                    f"ros2 bag startup failed: bag={output_path} "
                    f"error={error} log={log_path}"
                )
                self.stop_process()
                self.clear_process()
                return self.respond(
                    response, False, False, "StartFailed", output_path,
                    f"{error} (log: {log_path})"
                )

            self.get_logger().info(
                f"ros2 bag recording confirmed: bag={output_path} "
                f"topics={topics or 'ALL'} log={log_path}"
            )
            return self.respond(
                response, True, True, "LoggingStart", output_path, "Recording started."
            )

        if command == "LoggingStop":
            if self.bag_proc is None or self.bag_proc.poll() is not None:
                if self.bag_proc is not None:
                    self.clear_process()
                return self.respond(
                    response, True, False, "NotRunning", self.last_bag_path,
                    "No recording process is running."
                )

            output_path = self.current_bag_path
            stopped, error = self.stop_process()
            if not stopped:
                self.get_logger().error(
                    f"failed to stop ros2 bag: bag={output_path} error={error}"
                )
                return self.respond(
                    response, False, True, "StopFailed", output_path, error
                )

            return_code = self.bag_proc.returncode
            self.clear_process()
            metadata_exists = os.path.isfile(os.path.join(output_path, "metadata.yaml"))
            message = (
                "Recording stopped and metadata was finalized."
                if metadata_exists
                else "Recording stopped, but metadata.yaml was not found."
            )
            self.get_logger().info(
                f"ros2 bag recording stopped: bag={output_path} "
                f"return_code={return_code} metadata={metadata_exists}"
            )
            return self.respond(
                response, True, False, "LoggingStop", output_path, message
            )

        running = self.bag_proc is not None and self.bag_proc.poll() is None
        message = f"unknown logging command: {command}"
        self.get_logger().warning(message)
        return self.respond(
            response, False, running, "UnknownCommand",
            self.current_bag_path or self.last_bag_path, message
        )

    def monitor_recording(self):
        if self.bag_proc is None or self.bag_proc.poll() is None:
            return
        bag_path = self.current_bag_path
        code = self.bag_proc.returncode
        details = self.log_tail()
        self.get_logger().error(
            f"ros2 bag stopped unexpectedly: bag={bag_path} return_code={code}"
            + (f" details={details}" if details else "")
        )
        self.clear_process()

    def destroy_node(self):
        try:
            if self.bag_proc is not None and self.bag_proc.poll() is None:
                self.get_logger().info(
                    f"logging server shutting down; stopping bag={self.current_bag_path}"
                )
                self.stop_process()
        finally:
            self.clear_process()
        return super().destroy_node()


def main():
    rclpy.init()
    node = RosbagLoggingServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()