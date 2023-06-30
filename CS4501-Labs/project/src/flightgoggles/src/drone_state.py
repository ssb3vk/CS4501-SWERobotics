#!/usr/bin/env python
import rospy
import time
import copy
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, Vector3, Point, Pose
from std_msgs.msg import Int8, Bool


# A class to keep track of the quadrotors state
class DroneState(Enum):
  HOVERING = 1
  SCANNING = 2
  MOVING = 3


# Create a class which takes in a position and verifies it, keeps track of the state and publishes commands to a drone
class DroneStateController():

  # Node initialization
  def __init__(self):
    self.acceptance_range = rospy.get_param("/drone_state_node/acceptance_range", 0.5)
    self.scanning_duration = rospy.get_param("/drone_state_node/scanning_duration", 1)
    rospy.loginfo("starting drone_state with scanning duration of: " + str(self.scanning_duration))
    rospy.loginfo("starting drone_state with acceptance range of: " + str(self.acceptance_range))

    self.state = DroneState.SCANNING
    rospy.loginfo(str(rospy.get_name()) + ": Current State: SCANNING")

    # Stores where the drone is supposed to be 
    self.drone_destination = Vector3(x=0,y=0,z=3)
    # Create a point message that saves the drones current position
    self.drone_position = Pose()

    # Keeps track of whether the "drone_destination" was changed or not
    self.goal_changed = False
    self.map_done = False
    self.door_done = False

    self.goal_sub = rospy.Subscriber("/uav/input/position", Vector3, self.get_destination)
    self.pos_sub = rospy.Subscriber("/uav/sensors/gps", PoseStamped, self.get_drone_position)
    self.door_done_sub = rospy.Subscriber("/door_done", Bool, self.get_door_done)
    self.map_done_sub = rospy.Subscriber("/map_done", Bool, self.get_map_done)
    self.state_pub = rospy.Publisher("/drone_state", Int8, queue_size=1)
    

    self.drone_state = Int8()
    self.drone_state.data = 1
    # Call the mainloop of our class
    self.mainloop()

  def get_door_done(self, msg): 
    self.door_done = True
  
  def get_map_done(self, msg): 
    self.map_done = True

  def get_destination(self, msg):
    self.goal_changed = True
    self.drone_destination = msg

  def get_drone_position(self, msg): # gets the drone's "true" position
    temp = PoseStamped()
    temp = msg
    self.drone_position = temp.pose

  def calculateDistance(self, true_position, current_position): # we don't do sqrt here (too expensive)
    return (true_position.x - current_position.x)**2 + (true_position.y - current_position.y)**2

  def goalToString(self, msg):
    pos_str = "(" + str(msg.x) 
    pos_str += ", " + str(msg.y)
    pos_str += ", " + str(msg.z) + ")"
    return pos_str

  def processScanning(self):
    # wait for the self.scanning_duration variable and then switch to hovering
    # we also broadcast all the time so yeah. 
    self.state_pub.publish(2) # we publish that we are scanning
    # unlike the others we only publish this once and sleep
    if ( self.map_done and self.door_done ): 
      rospy.loginfo(str(rospy.get_name()) + ": Current State: HOVERING")
      self.state = DroneState.HOVERING
      self.map_done = self.door_done = False

  # This function is called when we are in the hovering state
  def processHovering(self):
    # Print the requested goal if the position changed
    self.state_pub.publish(1)
    if self.goal_changed:  
      rospy.loginfo(str(rospy.get_name()) + ": Requested Position: " + self.goalToString(self.drone_destination))
      rospy.loginfo(str(rospy.get_name()) + ": Current State: MOVING")
      self.state = DroneState.MOVING
      self.goal_changed = False


  # This function is called when we are in the moving state
  def processMoving(self):
    # Compute the distance between requested position and current position
    self.state_pub.publish(3)
    distance_to_goal = self.calculateDistance(self.drone_destination, self.drone_position.position)

    # print("Current Drone Position: \t(" + str(round(self.drone_position.position.x, 1)) + ", " + str(round(self.drone_position.position.y, 1)) + ", " + str(round(self.drone_position.position.z, 1)) + ")")
    # print("Current Waypoint: \t\t(" + str(self.drone_destination.x) + ", " + str(self.drone_destination.y) + ", " + str(self.drone_destination.z) + ")")
    # print("-------------------")

    # If goal is reached transition to scanning
    if distance_to_goal < self.acceptance_range:
      self.state = DroneState.SCANNING
      rospy.loginfo(str(rospy.get_name()) + ": Complete")
      rospy.loginfo(str(rospy.get_name()) + ": ----------------------------------")
      rospy.loginfo(str(rospy.get_name()) + ": Current State: SCANNING")


  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(20)

    # While ROS is still running
    while not rospy.is_shutdown():
      if self.state == DroneState.MOVING: 
        self.processMoving()
      if self.state == DroneState.SCANNING: 
        self.processScanning()
      if self.state == DroneState.HOVERING:
        self.processHovering() 

      # rospy.loginfo("drone dest is: " + str(self.drone_destination.position.x) + str(self.drone_destination.position.y) + str(self.drone_destination.position.z))
      # rospy.loginfo("drone's pos is: " + str(self.drone_position.position.x) + ", " +  str(self.drone_position.position.y) + ", " + str(self.drone_position.position.z) )
      # rospy.loginfo("this is goofy: " + str( (float(self.drone_destination.position.x) - float(self.drone_position.position.x))**2 ) )
      # rospy.loginfo("this is goofy: " + str( (float(self.drone_destination.position.y) - float(self.drone_position.position.y))**2 ) )
      # rospy.loginfo("this is goofy: " + str( (float(self.drone_destination.position.z) - float(self.drone_position.position.z))**2 ) )
      # rospy.loginfo("drone's distance is: " + str( self.calculateDistance(self.drone_destination.position, self.drone_position.position)) )

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('drone_state_node')
  try:
    ktp = DroneStateController()
  except rospy.ROSInterruptException:
    pass