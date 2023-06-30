#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import PoseStamped
from altitude.msg import AltitudeStamped
from moving_average import MovingAverage

class GPSAltitudeMA:
    # Node initialization
    def __init__(self):
        # Create the publisher and subscriber
        self.pub = rospy.Publisher('/uav/sensors/altitude_ma',
                                   AltitudeStamped,
                                   queue_size=1)
        self.moving_average = MovingAverage(5)
        self.sub = rospy.Subscriber('/uav/sensors/gps_noisy',
                                    PoseStamped, self.process_altitude,
                                    queue_size=1)
        self.gps_error = float(rospy.get_param('/uav/sensors/position_noise_lvl', str(0)))
        self.altitude = AltitudeStamped()
        self.gps_pose = PoseStamped()
        rospy.spin()

    def process_altitude(self, msg):
        self.gps_pose = msg
        self.moving_average.add(self.gps_pose.pose.position.z)
        # self.altitude.value = self.gps_pose.pose.position.z
        self.altitude.value = self.moving_average.get_average()
        self.altitude.stamp = self.gps_pose.header.stamp
        self.altitude.error = self.gps_error / math.sqrt(self.moving_average.window_size)
        self.pub.publish(self.altitude)


if __name__ == '__main__':
    rospy.init_node('gps_altitude_node')
    try:
        GPSAltitudeMA()
    except rospy.ROSInterruptException:
        pass