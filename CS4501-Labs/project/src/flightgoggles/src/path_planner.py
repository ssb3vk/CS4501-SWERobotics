#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Int8MultiArray, Int32MultiArray, Int8
from environment_controller.srv import use_key, use_keyResponse
from Queue import PriorityQueue, Queue
from enum import Enum

class DroneState(Enum):
  HOVERING = 1
  SCANNING = 2
  MOVING = 3

class PathPlanner():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)
    self.width = int(rospy.get_param("/path_planner_node/map_width", 23))
    self.height = int(rospy.get_param("/path_planner_node/map_height", 1))
    self.goal = []
    self.door_sub = rospy.Subscriber('/doors', Int32MultiArray, self.get_doors)

    rospy.sleep(3)
    # Set the rate
    self.rate = 10
    self.dt = 1.0 / self.rate

    # init class vars
    self.map =[]
    self.position = tuple()
    
    self.doors = []
    self.path = []
    self.state = DroneState.SCANNING

    # global a star planner vars
    self.global_q = PriorityQueue()
    self.graph = {}  # stores nodes and their parents
    self.distances  = {}  # tracks distance of position from start
    self.start = (0,0,3)
    # subs
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
    self.att_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
    self.state_sub = rospy.Subscriber('/drone_state', Int8, self.get_state)

    # pubs
    self.pos_pub = rospy.Publisher('uav/input/position', Vector3, queue_size=1)
    self.path_pub = rospy.Publisher('/uav/final_path', Int32MultiArray, queue_size=1)
    self.debug_path = rospy.Publisher('/uav/path', Int32MultiArray, queue_size=1)
    self.path_pub = rospy.Publisher('/uav/final_path', Int32MultiArray, queue_size=1)
    # initialize a star planner

    # local queue to get drone to wherever the global queue wants to explore next
    self.local_q = Queue()
    self.global_target = self.start
    self.local_target = self.start

    self.graph[self.start] = ""
    rospy.loginfo("goal {}".format(self.goal))
    f = (self.goal[0] - self.start[0])**2 + (self.start[1] - self.start[1])**2
    self.distances[self.start] = 0
    self.global_q.put((0,self.start))
    rospy
    rospy.loginfo("{} {} {}".format(self.global_target, self.local_target, self.position))
    rospy.sleep(2)
    # Run the node
    self.Run()

  def global_planner_astar(self):
    # rospy.loginfo("pos: "+str(self.position))
    # Check if the current node is the goal

     #  local goal available, move to it
    if self.local_q.qsize() > 0:
      target = self.local_q.get()
      pub = Vector3()
      pub.x = target[0]
      pub.y = target[1]
      pub.z = target[2]
      self.local_target = target
      rospy.loginfo("local planner next target: {}".format(target))
      self.pos_pub.publish(pub)

    #  reached global+local goal so find neighbors and pick a new global and local goal
    elif self.global_target == self.local_target == self.position:
      
      for direction in [(1,0), (0,1), (-1,0), (0,-1)]:
        # Compute the cost to traverse to that node
        g = self.distances[self.position] + 1
        neighbor = (self.position[0]+direction[0], self.position[1]+direction[1], 3)
        # check if neighboring pixel is empty or open door, and if its in the graph it should be lower cost
        if self.map[self.w2g(neighbor)] < 1 and (neighbor not in self.graph or g < self.distances[neighbor]):
          # Compute the estimated cost to goal (heuristic)
          h = (self.goal[0] - neighbor[0])**2 + (self.goal[1] - neighbor[1])**2
          if neighbor in self.doors:
            h-=1
          # Add the neighbors to the frontier with the new cost
          f = g + h
          rospy.loginfo("neighbor: {} prio: {}".format(neighbor, f))
          self.graph[neighbor] = self.position
          self.distances[neighbor] = g
          self.global_q.put((f,neighbor))
      
      # find the best global goal
      dist, node = self.global_q.get()
      self.global_target = node
      rospy.loginfo("A star global planner next target: {}".format(node))
     
      # check if the drone is 1 pixel away from this position
      if abs(self.position[0] - node[0]) + abs(self.position[1] - node[1]) < 2:
        #it is so put in local queue
        self.local_q.put(node)

      else:
        # had to back track so find path to the root of the fork
        path = self.generate_path(self.position, self.graph[node])
        for pos in path:
          self.local_q.put(pos)
        self.local_q.put(node)

    else:
      rospy.loginfo("something went wrong, ran out of places to explore but didn't find goal")

  def get_state(self, msg): 
    if ( DroneState(msg.data) != self.state ): 
      self.state = DroneState(msg.data)
      # rospy.loginfo("path planner node recieved state: {}".format(self.state))
  def get_doors(self, msg):
    
    self.goal = self.g2w(msg.data[0])
    rospy.loginfo("path planner recieved goal: {}, {} ".format(self.goal[0], self.goal[1]))
    if len(msg.data) > 1:
      self.doors = {self.g2w(i) for i in msg.data[1:]}

  def get_gps(self, msg):
    self.position = (int(round(msg.pose.position.x)), int(round(msg.pose.position.y)), 3)

  def get_map(self, msg):
    self.map = msg.data

  def g2w(self, index):
    # returns index grid corresponding to a x,y point in the world frame
    y = index % self.height - self.height // 2
    x = index / self.height - self.width  // 2

    return (x, y, 3)

  def w2g(self, coords):
    # returns index grid corresponding to a x,y point in the world frame
    return self.height * (int(round(coords[0])) + self.width // 2 ) + self.height//2 + int(round(coords[1]))

  def generate_path(self, start, end, final = False):
    path = []
    parent = self.position
    while parent != end:
      path.append((parent[0], parent[1], parent[2]))
      rospy.loginfo("path: {} index: {}".format(parent, self.w2g(parent)))
      parent = self.graph[parent]

    if final:
      path.append(self.start)
      path.reverse()
      return path
    else:
      if len(parent)>=3:
        path.append((parent[0], parent[1], parent[2]))
    return path

  

  def Run(self):
    rate = rospy.Rate(self.rate)

    while not rospy.is_shutdown():
      if self.state == DroneState.HOVERING:
        if self.position == self.goal:
          rospy.loginfo("made to goal")
          self.path = self.generate_path(self.goal, self.start, True)
          self.path = [self.w2g(i) for i in self.path]
          rospy.loginfo("path: {}".format(self.path))
          self.debug_path.publish(Int32MultiArray(data=self.path))
          rospy.sleep(5)
          self.path_pub.publish(Int32MultiArray(data=self.path))
        else:
          self.global_planner_astar()
      rate.sleep()

  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node("path_planner_node")
  try:
    MapGen = PathPlanner()
  except rospy.ROSInterruptException as e:
    rospy.loginfo("Node failed to start: %s"%e)


if __name__ == '__main__':
  main()