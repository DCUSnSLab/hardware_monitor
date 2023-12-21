#!/usr/bin/env python3
import time
import rospy
import sensor_msgs.msg
from std_msgs.msg import *
from hunter_msgs.msg import *
import psutil
import nvidia_smi
import socket
import rosnode


# class 예쁘게 만들기

class scvInformation:
    pubCpu = rospy.Publisher('/pubCpu', Float64, queue_size=10)
    pubRam = rospy.Publisher('/pubRam', Float64, queue_size=10)
    pubGpu = rospy.Publisher('/pubGpu', Float64, queue_size=10)
    pubScvIP = rospy.Publisher('/pubScvIP', String, queue_size=10)
    pubScvStatus = rospy.Publisher('/pubScvStatus', UInt64MultiArray, queue_size=10)

    def __init__(self):
        self.pubCpu.publish(self.cpuUsage())
        self.pubGpu.publish(self.gpuUsage())
        self.pubRam.publish(self.ramUsage())
        self.pubScvIP.publish(self.checkSCVIP())
        self.hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, self.callback)

    def callback(self, data):
        scvMsgArr = UInt64MultiArray()

        # scvMsgArr.data = [data.base_state, data.battery_voltage, data.control_mode,
        #                     data.fault_code, data.linear_velocity, data.park_mode, data.steering_angle,
        #                  data.motor_states[0], data.driver_states[0]]

        print(scvMsgArr.data)

        # ['MOTOR_ID_FRONT', 'MOTOR_ID_REAR_LEFT', 'MOTOR_ID_REAR_RIGHT', '__class__', '__delattr__', '__dir__',
        #  '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__getstate__', '__gt__', '__hash__',
        #  '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__',
        #  '__reduce_ex__', '__repr__', '__setattr__', '__setstate__', '__sizeof__', '__slots__', '__str__',
        #  '__subclasshook__', '_check_types', '_connection_header', '_full_text', '_get_types', '_has_header', '_md5sum',
        #  '_slot_types', '_type', 'base_state', 'battery_voltage', 'control_mode', 'deserialize', 'deserialize_numpy',
        #  'driver_states', 'fault_code', 'header', 'linear_velocity', 'motor_states', 'park_mode', 'serialize',
        #  'serialize_numpy', 'steering_angle']

        self.pubScvStatus.publish(scvMsgArr)

    def gpuUsage(self):
        nvidia_smi.nvmlInit()
        deviceCount = nvidia_smi.nvmlDeviceGetCount()
        for i in range(deviceCount):
            handle = nvidia_smi.nvmlDeviceGetHandleByIndex(i)
            util = nvidia_smi.nvmlDeviceGetUtilizationRates(handle)
            mem = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)
            print(
                f"|Device {i}| Mem Free: {mem.free / 1024 ** 2:5.2f}MB / {mem.total / 1024 ** 2:5.2f}MB "
                f"| gpu-util: {util.gpu / 100.0:3.1%} | gpu-mem: {util.memory / 100.0:3.1%} |")
            gpuUsageValue = util.gpu
            gpuMempercent = {util.memory / 100.0}
            print(gpuUsageValue)
            return gpuUsageValue

    def cpuUsage(self):
        # self.cpuUsageValue = cpuUsageValue
        cpuUsageValue = psutil.cpu_percent()
        print(f"CPU Percentage usage: {cpuUsageValue}%")
        # CPU frequency
        # cpu_info = psutil.cpu_freq()
        # print(f"CPU Current frequency: {cpu_info.current:.2f} Mhz")
        return cpuUsageValue

    def ramUsage(self):
        ram_info = psutil.virtual_memory()
        print(f"RAM Total: {ram_info.total / 1024 / 1024 / 1024:.2f} GB")
        print(f"RAM Available: {ram_info.available / 1024 / 1024 / 1024:.2f} GB")
        print(f"RAM Used: {ram_info.used / 1024 / 1024 / 1024:.2f} GB")
        print(f"RAM Percentage usage: {ram_info.percent}%")
        ramTotalGB = {ram_info.total / 1024 / 1024 / 1024: .2}
        ramAvailableGB = {ram_info.available / 1024 / 1024 / 1024: .2}
        ramUsedGB = {ram_info.used / 1024 / 1024 / 1024}
        ramUsageValue = ram_info.percent
        print(ramUsageValue)
        return ramUsageValue

    def checkSCVIP(self):
        # self.ipAddressValue = ipAddressValue
        hostname = socket.gethostname()
        ipAddressValue = socket.gethostbyname(hostname)
        print(hostname, ":", ipAddressValue)
        return ipAddressValue



if __name__ == '__main__':
    rospy.init_node('scvStatus', anonymous=True)
    # rospy.spin()

    while not rospy.is_shutdown():
        scvIn = scvInformation()
        time.sleep(1)