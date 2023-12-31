#!/usr/bin/env python
import sys

import rospy
import numpy as np

from gui import GUI, gui_shutdown_hook
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion


class GUI_Controller():

    def __init__(self):
        # When this node shutsdown
        rospy.on_shutdown(self.shutdown_sequence)

        # Set the rate
        self.rate = 2.0
        self.dt = 1.0 / self.rate

        # Create the position
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)

        gui_data = {'quad1': {'position': [0, 0, 0], 'orientation': [0, 0, 0], 'L': 1}}

        # Create the subscribers and publishers
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)

        # Create the gui object
        rospy.on_shutdown(gui_shutdown_hook)

        self.gui_object = GUI(quads=gui_data, rate=self.rate, yield_func=self.update_loop)

        rospy.spin()

    # This is the main loop of this class
    def update_loop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While running
        while not rospy.is_shutdown():
            # Display the position
            yield {
                'quad1':
                    {
                        'position': list(self.position),
                        'orientation': list(euler_from_quaternion(self.quaternion))
                    }
            }

            # Sleep any excess time
            rate.sleep()

    # Call back to get the gps data
    def get_gps(self, msg):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        self.quaternion = (msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w)

    # Called on ROS shutdown
    def shutdown_sequence(self):
        rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
    rospy.init_node('GUI_Controller_Node')
    try:
        v = GUI_Controller()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
