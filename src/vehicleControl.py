import rospy
from hardware_monitor.srv import VehicleControl
from geometry_msgs.msg import Twist

def handle_vehicle_control(req):

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    result = req.scvStatus

    startMsg = Twist()
    startMsg.linear.x = 1
    startMsg.linear.y = 0.0
    startMsg.linear.z = 0.0
    startMsg.angular.x = 0.0
    startMsg.angular.y = 0.0
    startMsg.angular.z = 0.0

    stopMsg = Twist()
    stopMsg.linear.x = 0.0
    stopMsg.linear.y = 0.0
    stopMsg.linear.z = 0.0
    stopMsg.angular.x = 0.0
    stopMsg.angular.y = 0.0
    stopMsg.angular.z = 0.0

    if str(result) == "scvStart":
        rospy.loginfo("scvStart")
        pub.publish(startMsg)
        return str(result)

    elif str(result) == "scvStop":
        rospy.loginfo("scvStop")
        pub.publish(stopMsg)
        return str(result)

    else:
        return 0

if __name__ == '__main__':

    rospy.init_node("SCV_Control_Server")
    rospy.loginfo("Vehicle Control Server start")
    service = rospy.Service("/scvControlServer", VehicleControl, handle_vehicle_control)
    rospy.loginfo("Service server has been started")

    rospy.spin()
