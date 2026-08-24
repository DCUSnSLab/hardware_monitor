#!/usr/bin/env python3
import os
import signal
import subprocess
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from hardware_monitor2_interfaces.srv import Logging


class RosbagLoggingServer(Node):
    def __init__(self):
        super().__init__("logging_server")

        # bag 저장 기본 경로: ~/hardware_monitor/bag (이동체마다 home이 달라도 expanduser로 해결)
        self.declare_parameter("bag_dir", os.path.expanduser("~/hardware_monitor/bag"))
        self.bag_dir = self.get_parameter("bag_dir").get_parameter_value().string_value

        self.srv = self.create_service(Logging, "/logging", self.handle_logging)
        self.bag_proc: Optional[subprocess.Popen] = None

        self.get_logger().info(f"logging_server ready. bag_dir={self.bag_dir}")

    def handle_logging(self, request: Logging.Request, response: Logging.Response):
        command = str(request.is_logging)

        if command == "LoggingStart":
            if self.bag_proc is not None and self.bag_proc.poll() is None:
                self.get_logger().info("ros2 bag is already recording.")
                response.logging_status = "AlreadyRecording"
                return response

            # 저장 경로 준비: ~/hardware_monitor/bag/<bag_name>
            os.makedirs(self.bag_dir, exist_ok=True)

            bag_name = (request.bag_name or "").strip()
            if not bag_name:
                bag_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            output_path = os.path.join(self.bag_dir, bag_name)

            # rosbag2는 output 경로가 이미 있으면 실패하므로 중복 시 이름 뒤에 시각을 붙인다.
            if os.path.exists(output_path):
                output_path = f"{output_path}_{datetime.now().strftime('%H-%M-%S')}"

            # 선택된 토픽 목록(빈 값 제거). 없으면 전체(-a)
            topics = [t for t in (request.topics or []) if t]

            cmd = ["ros2", "bag", "record", "-o", output_path]
            if topics:
                cmd += topics
            else:
                cmd.append("-a")

            try:
                self.bag_proc = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                )
                self.get_logger().info(
                    f"ros2 bag record started: out={output_path} topics={topics or 'ALL'}"
                )
                response.logging_status = "LoggingStart"
            except Exception as e:
                self.get_logger().error(f"failed to start ros2 bag: {e}")
                response.logging_status = "StartFailed"
            return response

        elif command == "LoggingStop":
            self.get_logger().info("LoggingStop requested")
            if self.bag_proc is None or self.bag_proc.poll() is not None:
                response.logging_status = "NotRunning"
                return response

            try:
                pgid = os.getpgid(self.bag_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                self.get_logger().info(f"sent SIGINT to ros2 bag pgid={pgid}")
                self.bag_proc.wait(timeout=10)
                response.logging_status = "LoggingStop"
            except Exception as e:
                self.get_logger().error(f"failed to stop ros2 bag: {e}")
                response.logging_status = "StopFailed"
            finally:
                if self.bag_proc and self.bag_proc.poll() is not None:
                    self.bag_proc = None
            return response

        else:
            self.get_logger().warn(f"unknown command: {command}")
            response.logging_status = "UnknownCommand"
            return response

    def destroy_node(self):
        try:
            if self.bag_proc and self.bag_proc.poll() is None:
                pgid = os.getpgid(self.bag_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                self.bag_proc.wait(timeout=5)
        except Exception:
            pass
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
