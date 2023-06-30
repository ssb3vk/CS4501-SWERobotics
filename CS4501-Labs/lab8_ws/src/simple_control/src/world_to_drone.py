#!/usr/bin/env python
import rospy
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_point


class WorldToDrone:

    def __init__(self):
        time.sleep(10)
        # Used by the callback for the topic /tower/goal
        self.gps = None
        self.transform_stamped = TransformStamped()
        # TODO: Instantiate the Broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        

        # TODO: Drone GPS subscriber to topic /uav/sensors/gps
        self.drone_gps_sub = rospy.Subscriber("/uav/sensors/gps", PoseStamped, self.drone_gps_sub_callback)
        
        # TODO: fill in the parent and child frames
        self.transform_stamped.header.frame_id = 'world'
        self.transform_stamped.child_frame_id = 'drone'

        # start main loop
        self.mainloop()

    # TODO: Callback for the GPS
    def drone_gps_sub_callback(self, msg): 
        self.gps = msg

    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.gps:
                # TODO: Use the GPS position to set up the transform
                # self.transform_stamped.transform.translation = None  # TODO: set the Translation from the GPS
                # self.transform_stamped.transform.rotation = None  # TODO: set the quaternion from the GPS

                # TODO: Update the header to the current timestamp
                self.transform_stamped.header.stamp = rospy.Time()

                # fill in parent frame, child frame, timestamp, and transform fields
                
                # might have to reverse all of these: 
                rospy.loginfo(str(rospy.get_name()) + "gps: " + str(self.gps.pose.position.x))
                rospy.loginfo(str(rospy.get_name()) + "transform_stamped: " + str(self.transform_stamped.transform.translation.x))
                self.transform_stamped.transform.translation.x = self.gps.pose.position.x
                self.transform_stamped.transform.translation.y = self.gps.pose.position.y
                self.transform_stamped.transform.translation.z = 0
                quat = quaternion_from_euler(float(-3.14159265359), float(-3.14159265359), float(0))
                self.transform_stamped.transform.rotation.x = quat[0]
                self.transform_stamped.transform.rotation.y = quat[1]
                self.transform_stamped.transform.rotation.z = quat[2]
                self.transform_stamped.transform.rotation.w = quat[3]

                # self.transform_stamped.transform.rotation.x = self.gps.pose.orientation.x
                # self.transform_stamped.transform.rotation.y = self.gps.pose.orientation.y
                # self.transform_stamped.transform.rotation.z = self.gps.pose.orientation.z
                # self.transform_stamped.transform.rotation.w = self.gps.pose.orientation.w

                self.br.sendTransform(self.transform_stamped)

                # TODO: Broadcast the transform
                self.transform_stamped.header.stamp = 0  # TODO: set to the current time


            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('world_to_drone')
    try:
        wod = WorldToDrone()
    except rospy.ROSInterruptException:
        pass