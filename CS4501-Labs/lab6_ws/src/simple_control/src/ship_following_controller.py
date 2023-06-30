#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from threading import Lock

from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from velocity_pid_class import PID

# A class used to follow ship below drone
class ShipFollower():
  # On node initialization
  def __init__(self):

    # Allow the simulator to start
    time.sleep(3)

    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # TODO: Retrieve rate from ROS params
    self.rate = rospy.get_param('/ship_following_controller_node/rate', 4)
    self.dt = 1.0 / self.rate


    # TODO: Retrieve the PID parameters from ROS parameters
    pid_params = rospy.get_param('/ship_following_controller_node/pid', {'p': 5, 'i': 5, 'd': 5})    
    p, i, d  = pid_params['p'], pid_params['i'], pid_params['d']

    zpid_params = rospy.get_param('/ship_following_controller_node/zpid', {'p': 5, 'i': 5, 'd': 5})    
    pz, iz, dz  = zpid_params['p'], zpid_params['i'], zpid_params['d']

    # TODO: Initialize PIDs with PID class and ROS parameters
    self.x_pid = PID(p, i, d)
    self.y_pid = PID(p, i, d)
    self.z_pid = PID(pz, iz, dz)


    # TODO: Initialize zero'ed class vars
    self.gps_height_value = 15 # where the drone should be
    self.drone_height = 0
    

    # TODO: Create the publishers and subscribers
    self.vel_pub = rospy.Publisher('/uav/input/velocityPID', Vector3, queue_size=1)
    self.gps_height = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.get_gps, queue_size=1)
    self.beacon1 = rospy.Subscriber('/ship/beacon1', Vector3Stamped, self.get_ship_beacon1, queue_size=1)
    self.beacon2 = rospy.Subscriber('/ship/beacon2', Vector3Stamped, self.get_ship_beacon2, queue_size=1)
    self.beacon1_data = Vector3()
    self.beacon2_data = None
    self.beacon1_time_stamp = 0
    self.beacon2_time_stamp = 0

    self.velocity_to_pub = Vector3()


    self.est_ship_pos = rospy.Publisher('ship/estimated_position', Vector3, queue_size=1)


    rospy.loginfo("Parameters are: " + str(p) + ", " + str(i) +  ", " + str(d))
    # Run the control loop
    self.ControlLoop()

  # TODO FOR CHECKPOINT 1
  def get_gps(self, msg):
    # ONLY SAVE THE HEIGHT
    self.drone_height = msg.pose.position.z
    pass

  # TODO FOR CHECKPOINT 2
  # callback for /ship/beacon1
  def get_ship_beacon1(self, msg):
    self.beacon1_time_stamp = msg.header.stamp
    self.beacon1_data.x = msg.vector.x / 1.33
    self.beacon1_data.y = msg.vector.y / 1.33
    pass

  # TODO FOR CHECKPOINT 2
  # callback for /ship/beacon2
  def get_ship_beacon2(self, msg):
    self.beacon2_data = Vector3()
    self.beacon2_time_stamp = msg.header.stamp
    self.beacon2_data.x = msg.vector.x / 3
    self.beacon2_data.y = msg.vector.y / 3
    pass

  # TODO FOR CHECKPOINT 3
  # combine PID output from the two beacons
  # You don't have to use this method,
  # but you will need to use both error terms
  def combine_beacons(self):
    pass

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # While running
    while not rospy.is_shutdown():
      # Use PIDs to calculate the velocities you want
      # TODO FOR CHECKPOINT 1: z velocity
      self.velocity_to_pub.z = self.z_pid.pid_loop( -1 * self.gps_height_value + self.drone_height, self.dt)
      rospy.loginfo("height error is: " + str(self.gps_height_value - self.drone_height))
      # TODO FOR CHECKPOINT 2: x and y velocity
      if self.beacon2_time_stamp > self.beacon1_time_stamp: 
        self.velocity_to_pub.x = self.x_pid.pid_loop(self.beacon2_data.x, self.dt)
        self.velocity_to_pub.y = self.y_pid.pid_loop(self.beacon2_data.y, self.dt)

      else: 
        self.velocity_to_pub.x = self.x_pid.pid_loop(self.beacon1_data.x, self.dt)
        self.velocity_to_pub.y = self.y_pid.pid_loop(self.beacon1_data.y, self.dt)

      # Sleep any excess time
      rate.sleep()

  # Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


if __name__ == '__main__':
  rospy.init_node('ship_follower_node')
  try:
    ktp = ShipFollower()
  except rospy.ROSInterruptException:
    pass
