import rclpy
import nvidia_smi
import psutil
import socket

from rclpy.node import Node
from std_msgs.msg import String, Float64

class ScvInformation(Node):
    def __init__(self):
        super().__init__('SCV_status')
        self.pub_cpu = self.create_publisher(Float64, '/pubCpu', 10)
        self.pub_ram = self.create_publisher(Float64, '/pubRam', 10)
        self.pub_gpu = self.create_publisher(Float64, '/pubGpu', 10)
        self.pub_ip = self.create_publisher(String , '/pubScvIP', 10)

        self.ip_cached = self.get_ip()

        self.timer = self.create_timer(1.0, self.tick)

    def cpu_usage(self):
        cpu_usage_value = psutil.cpu_percent()
        return cpu_usage_value

    def gpu_usage(self):
        nvidia_smi.nvmlInit()
        deviceCount = nvidia_smi.nvmlDeviceGetCount()
        for i in range(deviceCount):
            handle = nvidia_smi.nvmlDeviceGetHandleByIndex(i)
            util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
            mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
            gpuUsageValue = util.gpu
            gpuMempercent = util.memory / 100.0
            return float(gpuUsageValue)

    def ram_usage(self):
        ram_info = psutil.virtual_memory()
        ramTotalGB = {ram_info.total / 1024 / 1024 / 1024: .2}
        ramAvailableGB = {ram_info.available / 1024 / 1024 / 1024: .2}
        ramUsedGB = {ram_info.used / 1024 / 1024 / 1024}
        ram_usage_value = ram_info.percent
        return ram_usage_value

    def get_ip(self):
        ip_address_value = socket.gethostbyname(socket.gethostname())
        return ip_address_value
    
    def tick(self):
        self.pub_cpu.publish(Float64(data=self.cpu_usage()))
        self.pub_ram.publish(Float64(data=self.ram_usage()))
        self.pub_gpu.publish(Float64(data=self.gpu_usage()))
        self.pub_ip.publish(String(data=self.ip_cached))

    def destroy_node(self):
        try:
            nvidia_smi.nvmlShutdown()
        except Exception as e:
            self.get_logger().warn(f"NVML shutdown failed or not initialized: {e}")

        try:
            if hasattr(self, 'timer') and self.timer is not None:
                self.timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Timer cancel failed: {e}")

        self.pub_cpu = None
        self.pub_ram = None
        self.pub_gpu = None
        self.pub_ip = None

def main():
    rclpy.init()
    node = ScvInformation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()