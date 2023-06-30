#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Pose, Vector3, PoseStamped, Point, PointStamped
from enum import Enum
import numpy as np
from std_msgs.msg import Int8MultiArray, Int8, Int32MultiArray, Bool
from environment_controller.srv import use_key, use_keyResponse
from tf2_geometry_msgs import do_transform_point
import tf2_ros
# TODO: Set door detection max distance????

# this file will look at the drone location
# and the lidar data 
# and tell us if there is a door there or not
# this should look as far as possible to detect a door
# can we detect a door with only one point? 
# doing so would be very nice
# but for now: detecting a door with a full side of points (3)
# need to get distance values from the drone's assumed location
# and then compute, otherwise everything jitters
# for door detection we only care about up to a value of 1. Otherwise the laser fluctuates
# too wildly

# a couple design principles: 
# clear data lists once we move from the point
# have a general mode of detecting when the drone is still/moving
# first things first we'll detect when the drone is still moving
# only detect door when its a distance of 1 away

class DroneState(Enum):
  HOVERING = 1
  SCANNING = 2
  MOVING = 3


class DoorDetector():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)
    self.drone_position_was_pubbed = False
    self.drone_laser_was_pubbed = False

    # Set the rate
    self.rate = 20
    self.dt = 1.0 / self.rate

    self.width = int(rospy.get_param("/map_generator_node/map_width", 23))
    self.height = int(rospy.get_param("/map_generator_node/map_height", 1))

    self.drone_real_position = Pose()
    self.drone_position = Vector3()

    self.laser = LaserScan()
    self.laser.ranges = 16 * [0]

    self.laser_history = []
    self.samples = int(rospy.get_param("/door_detector_node/samples", 100)) # enables fine tuning for samples to detect that we have a door or not

    self.distance_history = []

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.doors = []  # first index is goal then rest are doors
    self.got_goal = False

    self.state = DroneState.HOVERING

    # Create the subscribers and publishers
    self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_laser)
    self.pos_sub = rospy.Subscriber('/uav/input/position', Vector3, self.get_pos)
    self.real_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.get_real_pos)
    self.state_sub = rospy.Subscriber('/drone_state', Int8, self.get_state)
    self.dog_sub = rospy.Subscriber("/cell_tower/position", Vector3, self.get_goal)

    self.door_done = rospy.Publisher('/door_done', Bool, queue_size=1)
    self.door_pub = rospy.Publisher('/doors', Int32MultiArray, queue_size=1)

    self.service = rospy.Service('use_key_service', use_key, self.use_key)

    # rospy.loginfo("we need" + str(self.samples) + "samples")

    # Run the node
    self.Run()

  def w2g(self, coords):
    # returns index grid corresponding to a x,y point in the world frame
    return self.height * (int(round(coords[0])) + self.width // 2 ) + self.height//2 + int(round(coords[1]))

  def get_goal(self, msg):
    if not self.got_goal:
      rospy.sleep(3)
      transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())

      goal = PointStamped()
      goal.point.x = msg.x
      goal.point.y = msg.y
      goal.point.z = msg.z

      goal.header.stamp = rospy.Time()
      # Use the do_transform_point function to convert the point using the transform
      goal = do_transform_point(goal, transform)

      self.doors.append(self.w2g([goal.point.x+0.5, goal.point.y+0.5]))
      self.door_pub.publish(Int32MultiArray(data=self.doors))
      self.got_goal = True
      rospy.loginfo("door detector recieved goal: {}, {}".format(goal.point.x+0.5, goal.point.y+0.5))

  def get_state(self, msg): 
    if ( DroneState(msg.data) != self.state ): 
      self.state = DroneState(msg.data)
      self.distance_history = []
      # rospy.loginfo("door detector node recieved state: {}".format(self.state))

  def use_key(self, doorLocation):
    rospy.wait_for_service('use_key')
    use_key_holder = rospy.ServiceProxy('use_key', use_key)
    try: 
      use_key_holder = rospy.ServiceProxy('use_key', use_key)
      response = use_key_holder(doorLocation)
    except rospy.ServiceException as exc:
      rospy.loginfo("service call failed: "+ str(exc))
    
    rospy.loginfo("able to open door: " + str(response))
    if response:
      self.doors.append(self.w2g([doorLocation.x, doorLocation.y]))
      self.door_pub.publish(Int32MultiArray(data=self.doors))
    return response

  def get_pos(self, msg): 
    self.drone_position = msg

  def get_laser(self, msg):
    self.drone_laser_was_pubbed = True
    self.laser = msg

  def get_real_pos(self, msg): 
    self.drone_position_was_pubbed = True
    temp = PoseStamped()
    temp = msg
    self.drone_real_position = temp.pose

  def print_history(self, history): 
    rospy.loginfo("history is:")
    for i in history: 
      rospy.loginfo(i)

  def Run(self):
     # Set the rate
    rate = rospy.Rate(self.rate)

    directions = [[0, 1], 
                  [-1, 0], 
                  [0, -1], 
                  [1, 0]]

    self.distance_history = []
    # While running
    while not rospy.is_shutdown():
      # if ( len(self.laser_history) > 0): 
      #   rospy.loginfo(str(np.std(self.laser_history, axis=0)))
      # # rospy.loginfo(self.laser_history)
      if self.state == DroneState.SCANNING: 
        # rospy.loginfo(self.laser.ranges)
        # checking if we got a "real" position and laser range
        if ( self.drone_laser_was_pubbed and self.drone_position_was_pubbed ): 
          self.distance_history.append(
                  [self.drone_real_position.position.y + min(self.laser.ranges[3], 5),
                   self.drone_real_position.position.x - min(self.laser.ranges[7], 5), 
                   self.drone_real_position.position.y - min(self.laser.ranges[11], 5),  
                   self.drone_real_position.position.x + min(self.laser.ranges[15], 5)
                   ]
                )
          
        # rospy.loginfo(temp[len(temp) - 1])
        # rospy.loginfo(np.std(distance_history, axis = 0))

        if ( len(self.distance_history) > self.samples ): 
          # rospy.loginfo(distance_history)
          # self.print_history(self.distance_history)
          means = np.mean(self.distance_history, axis = 0)
          stddev = np.std(self.distance_history, axis = 0)
          # rospy.loginfo(self.distance_history)
          # rospy.loginfo(means)
          # rospy.loginfo(stddev)

          for i in range(len(stddev)): 
            # abs(means[i]) < 1  and (may or may not need to be in the if statement)
            if ( stddev[i] > 0.02 and self.laser.ranges[i * 4 + 3] < 1 ): # can be set up to 0.03 if needed also only look at doors that are dist 1 away
              temp = Point()
              temp.x = self.drone_position.x + directions[i][0] * math.ceil(self.laser.ranges[i * 4 + 3])
              temp.y = self.drone_position.y + directions[i][1] * math.ceil(self.laser.ranges[i * 4 + 3])
              rospy.loginfo("direction " + str(i) + " has a door at position " + str(temp))
              self.use_key(temp)
          self.state = DroneState.HOVERING

          self.door_done.publish(True)
        
      rate.sleep()

  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node("door_detector_node")
  try:
    DoorDet = DoorDetector()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()