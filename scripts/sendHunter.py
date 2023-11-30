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

cpuPub = rospy.Publisher('/cpuPub', Float64, queue_size=10)
ramPub = rospy.Publisher('/ramPub', Float64, queue_size=10)
gpuPub = rospy.Publisher('/gpuPub', Float64, queue_size=10)
scvIPPub = rospy.Publisher('/scvIPPub', String, queue_size=10)
scvStatusPub = rospy.Publisher('/scvStatusPub', Float32MultiArray, queue_size=10)
imgSamplingPub = rospy.Publisher('/imgSamplingPub', sensor_msgs.msg.CompressedImage, queue_size=10)

def gpuUsage():
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
        gpuPub.publish(gpuUsageValue)

def cpuUsage():
    # self.cpuUsageValue = cpuUsageValue
    cpuUsageValue = psutil.cpu_percent()
    print(f"CPU Percentage usage: {cpuUsageValue}%")
    # CPU frequency
    # cpu_info = psutil.cpu_freq()
    # print(f"CPU Current frequency: {cpu_info.current:.2f} Mhz")
    cpuPub.publish(cpuUsageValue)

def ramUsage():
    # self.ramUsageValue = ramUsageValue
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
    ramPub.publish(ramUsageValue)

def checkSCVIP():
    # self.ipAddressValue = ipAddressValue
    hostname = socket.gethostname()
    ipAddressValue = socket.gethostbyname(hostname)
    print(hostname, ":", ipAddressValue)
    scvIPPub.publish(ipAddressValue)


# convert json to ros msg
def checkSCVstatus():
    floatMsgArr = Float32MultiArray()

    # nodes = rosnode.get_node_names()`
    # scv_node_name = '/play'
    # scv_ID = 0
    #
    # for node in nodes:
    #     if(node.startswith(scv_node_name)):
    #         scv_ID += 1
    def callback(data):
        # print(rospy.get_caller_id(), data)
        floatMsgArr.data = [data.control_mode, data.base_state, data.park_mode, data.battery_voltage, data.linear_velocity, data.steering_angle]
        scvStatusPub.publish(floatMsgArr)
    hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, callback)

# def samplingImg():
#     def callback(data):
#         # print(rospy.get_caller_id(), data)
#         floatMsgArr.data = [data.control_mode, data.base_state, data.park_mode, data.battery_voltage, data.linear_velocity, data.steering_angle]
#         scvStatusPub.publish(floatMsgArr)
#     hunterStatusSub = rospy.Subscriber('/hunter_status', HunterStatus, callback)


if __name__ == '__main__':
    rospy.init_node('scvStatus', anonymous=True)
    # rospy.spin()

    while not rospy.is_shutdown():
        checkSCVIP()
        gpuUsage()
        cpuUsage()
        ramUsage()
        checkSCVstatus()
        time.sleep(1)