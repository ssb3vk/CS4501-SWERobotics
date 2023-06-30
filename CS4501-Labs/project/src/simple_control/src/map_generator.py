#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Pose, PoseStamped, Vector3, PointStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from enum import Enum
from std_msgs.msg import Int8, Int32MultiArray, Bool, Int32

from tf2_geometry_msgs import do_transform_point
import tf2_ros

FREE = 0
BLOCK = 100
DOOR = -2
UNKOWN = 50

class DroneState(Enum):
  HOVERING = 1
  SCANNING = 2
  MOVING = 3

class MapGenerator():
  #generates map based on lidar readings 
  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 20
    self.dt = 1.0 / self.rate

    self.map = OccupancyGrid()
    self.map.info.width = self.width = int(rospy.get_param("/map_generator_node/map_width", 23))
    self.map.info.height = self.height = int(rospy.get_param("/map_generator_node/map_height", 23))
    self.map.info.resolution = 1
    origin = Pose()
    origin.position.x = 0.5 + -1 * self.map.info.width / 2 
    origin.position.y = 0.5 + -1 * self.map.info.height / 2
    origin.position.z = 0
    self.map.info.origin = origin


    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # initializing occupancy grid to all 50's
    self.map.data = [UNKOWN for i in range(self.width * self.height)]

    # lidar
    self.laser = LaserScan()
    self.laser.ranges = 16 * [0]

    # drone coords:
    self.x = 0
    self.y = 0

    self.state = DroneState.HOVERING

    #door finding 
    self.doors = []
    self.goal = 0
    self.got_goal = False

    # sub
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
    self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_laser)
    self.state_sub = rospy.Subscriber('/drone_state', Int8, self.get_state)
    self.door_sub = rospy.Subscriber('/doors', Int32MultiArray, self.get_doors)
    self.goal_sub = rospy.Subscriber('/goal', Int32, self.get_goal)
    self.dog_sub = rospy.Subscriber("/cell_tower/position", Vector3, self.get_goal)
    # pub
    self.map_done = rospy.Publisher('/map_done', Bool, queue_size=1)
    self.att_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    self.pos_pub = rospy.Publisher('uav/input/position', Vector3, queue_size=1)
    # Run the node
    self.Run()

  def get_goal(self, msg):
    if not self.got_goal:
      rospy.sleep(1)
      transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())

      goal = PointStamped()
      goal.point.x = msg.x
      goal.point.y = msg.y
      goal.point.z = msg.z

      goal.header.stamp = rospy.Time()
      # Use the do_transform_point function to convert the point using the transform
      goal = do_transform_point(goal, transform)

      self.got_goal = True
      self.goal = self.w2g(goal.point.x+0.5, goal.point.y+0.5)
      self.map.data[0] = 50
      rospy.loginfo("map gen recieved goal: {}, {}".format(goal.point.x+0.5, goal.point.y+0.5))

  def get_doors(self, msg):
    self.doors = msg.data
      

  def get_state(self, msg): 
    if ( DroneState(msg.data) != self.state ): 
      self.state = DroneState(msg.data)

  def get_laser(self, msg):
    self.laser = msg

  def get_gps(self, msg):
    self.x = int(round(msg.pose.position.x))
    self.y = int(round(msg.pose.position.y))

  def w2g(self, x, y):
    # returns index grid corresponding to a x,y point in the world frame
    return self.height * (int(round(x)) + self.width // 2 ) + self.height//2 + int(round(y))

  def Run(self):
    rate = rospy.Rate(self.rate)

    while not rospy.is_shutdown():
      if ( self.state == DroneState.SCANNING or self.state == DroneState.HOVERING): 
        '''
        in all 4 direction, the laser range will give the edge where a block is so the block is a bit past that edge
        if laser reading is 3.5, then obstacle is 4 blocks away so mark 0-3 spaces as free and 4th block in that direction as obstacle
        if the laser reading is inf, then dont mark anything as obstacle
        '''
        # right                                                                
        r = int(round(self.laser.ranges[15] +0.5)) if self.laser.ranges[15] < 5 else 6
        for i in range(min(r,5)):
            self.map.data[self.w2g(self.x+i,self.y)] = FREE
        if r != 6:
          self.map.data[self.w2g(r+self.x,self.y)] = BLOCK
        
        # up
        u = int(round(self.laser.ranges[3] +0.5)) if self.laser.ranges[3] < 5 else 6
        for i in range(min(u,5)):
            self.map.data[self.w2g(self.x,self.y+i)] = FREE
        if u != 6:
          self.map.data[self.w2g(self.x,self.y+u)] = BLOCK

        # left
        l = int(round(self.laser.ranges[7] + 0.5)) if self.laser.ranges[7] < 5 else 6
        for i in range(min(l,5)):
            self.map.data[self.w2g(self.x-i,self.y)] = FREE
        if l !=6:
          self.map.data[self.w2g(self.x-l,self.y)] = BLOCK
        
        # down
        d = int(round(self.laser.ranges[11] + 0.5)) if self.laser.ranges[11] < 5 else 6
        for i in range(min(d,5)):
            self.map.data[self.w2g(self.x,self.y-i)] = FREE
        if d != 6:
          self.map.data[self.w2g(self.x,self.y-d)] = BLOCK

      # mark doors and goal
      for i, val in enumerate(self.doors):
        self.map.data[val] = -2
      self.map.data[self.goal] = -3
      


      self.att_pub.publish(self.map)
      self.map_done.publish(True)
      # Sleep any excess time
      rate.sleep()

  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node("map_generator_node")
  try:
    MapGen = MapGenerator()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()