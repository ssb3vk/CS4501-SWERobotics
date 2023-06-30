#!/usr/bin/env python
from hashlib import new
import rospy
import tf2_ros
import time
import copy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point


class GroundRobotController:

    def __init__(self):
        time.sleep(20)  # takes longer for the other items to come up
        # Our goal is 0,0,0 *in the drone frame*
        self.goal = Vector3()
        # TODO: Instantiate the Buffer and TransformListener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # TODO: set up publisher to /ground_robot/goal - this will publish *in the world frame*
        self.robot_goal_pub = rospy.Publisher('/ground_robot/goal', Vector3, queue_size=1)

        # start main loop
        self.mainloop()


    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.goal:

                try:
                    # TODO: Lookup the drone to world transform
                    transform = self.tfBuffer.lookup_transform('drone', 'world', rospy.Time())
                    # TODO: Convert the goal to a PointStamped
                    point_stamped_goal = PointStamped()
                    point_stamped_goal.point = self.goal
                    # TODO: Use the do_transform_point function to convert the point using the transform
                    new_point = do_transform_point(point_stamped_goal, transform)
                    # TODO: Convert the point back into a vector message containing integers
                    msg = Vector3()
                    msg.x = new_point.point.x
                    msg.y = new_point.point.y
                    # TODO: Publish the vector
                    rospy.loginfo(str(rospy.get_name()) + ": Publishing Transformed Goal {}".format([msg.x, msg.y]))
                    self.robot_goal_pub.publish(msg)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ground_robot_controller')
    try:
        tom = GroundRobotController()
    except rospy.ROSInterruptException:
        pass