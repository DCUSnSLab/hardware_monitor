#!/usr/bin/env python3
import os
import signal
import subprocess
import rclpy

from typing import Optional
from rclpy.node import Node
from hardware_monitor2_interfaces.srv import Logging


class RosbagLoggingServer(Node):
    def __init__(self):
        super().__init__("logging_server")
        self.srv = self.create_service(Logging, "/logging", self.handle_logging)
        self.bag_proc: Optional[subprocess.Popen] = None

    def handle_logging(self, request: Logging.Request, response: Logging.Response):
        logging_status = str(request.is_logging)

        if logging_status == "LoggingStart":
            if self.bag_proc is not None and self.bag_proc.poll() is None:
                self.get_logger().info("ros2 bag is already recording.")
                response.result = "AlreadyRecording"
                return response
            
            try:
                self.bag_proc = subprocess.Popen(
                    ["ros2", "bag", "record", "-a"],
                    preexec_fn = os.setsid,
                    stdout = subprocess.DEVNULL,
                    stderr = subprocess.STDOUT
                )
                response.result = "LoggingStart"
            except Exception as e:
                self.get_logger().error(f"failed to start ros2 bag: {e}")
                response.result = "StartFailed"
            return response

        elif logging_status == "LoggingStop":
            self.get_logger().info("LoggingStop requested")
            if self.bag_proc is None or self.bag_proc.poll() is not None:
                response.result = "NotRunning"
                return response

            try:
                pgid = os.getpgid(self.bag_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                self.get_logger().info(f"sent SIGINT to ros2 bag pgid={pgid}")
                self.bag_proc.wait(timeout=10)
                response.result = "LoggingStop"
            except Exception as e:
                self.get_logger().error(f"failed to stop ros2 bag: {e}")
                response.result = "StopFailed"
            finally:
                if self.bag_proc and self.bag_proc.poll() is not None:
                    self.bag_proc = None
            return response

        else:
            self.get_logger().warn(f"unknown command: {logging_status}")
            response.result = "UnknownCommand"
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