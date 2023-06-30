#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from altitude.msg import AltitudeStamped
from visualization_msgs.msg import Marker


class AltitudeVisualizer:
    # Node initialization
    def __init__(self):
        # Create the publisher and subscriber
        self.marker_pub = rospy.Publisher('/uav/visualizer/altitude_marker',
                                   Marker,
                                   queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude',
                                    AltitudeStamped, self.process_altitude,
                                    queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude_ma',
                                    AltitudeStamped, self.process_altitude_ma,
                                    queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude_fused_ma',
                                    AltitudeStamped, self.process_altitude_fused_ma,
                                    queue_size=1)

        self.sub = rospy.Subscriber('/uav/sensors/altitude_kalman',
                                    AltitudeStamped, self.process_altitude_kalman,
                                    queue_size=1)
        rospy.spin()

    def process_altitude(self, msg):
        self.generate_marker(msg, 0, (1, 1, 0))  # yellow

    def process_altitude_ma(self, msg):
        self.generate_marker(msg, 1, (0, 1, 0))  # green

    def process_altitude_fused_ma(self, msg):
        self.generate_marker(msg, 2, (1, 0, 0))  # red

    def process_altitude_kalman(self, msg):
        self.generate_marker(msg, 3, (1, 0, 1))  # magenta

    def generate_marker(self, msg, id, color):
        # the Marker message type is provided by RViz and contains
        # all of the fields that we need to set in order for it to
        # display our shapes.
        marker = Marker()
        # The frame_id string tells RViz what frame of reference to
        # use when drawing this Marker. In this case, the "imu_ground"
        # frame is generated by the provided lab code and has its
        # origin set on the xy-plane directly below the robot
        marker.header.frame_id = "uav/imu_ground"
        marker.header.stamp = msg.stamp
        marker.ns = "uav"
        # If you have multiple Markers, they each need a unique id.
        # Here, each of the different topics we are listening to 
        # has its own id hard coded into the method call
        marker.id = id
        # RViz has several built in shapes, see the documentation
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # Below we set the location of the Marker. The "imu_ground"
        # reference frame has its origin directly below the robot, 
        # so to have the Marker appear at the altitude of the robot,
        # we set the z position to be the altitude message value
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = msg.value
        # Quaternions represent rotation. The below quaternion is
        # the "no rotation" quaternion. Since we have a sphere object
        # the rotation does not really matter.
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # This is the radius of the sphere. We want the size of the 
        # sphere to represent the error in the measurement. The
        # larger the sphere, the less certain we are. If there are
        # multiple spheres, we can think of the volume of each sphere
        # as representing the 95% confidence interval for that value
        marker.scale.x = msg.error
        marker.scale.y = msg.error
        marker.scale.z = msg.error
        # The color is represented in (a,r,g,b). The alpha=0.5 tells
        # RViz to make the Marker semi-transparent. We do this so that
        # if two Markers intersect we can still see both. The color is
        # passed in as an argument to this function so that each topic
        # has its own color. You can refer above for the color mapping.
        marker.color.a = 0.5
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('altitude_visualization_node')
    try:
        AltitudeVisualizer()
    except rospy.ROSInterruptException:
        pass