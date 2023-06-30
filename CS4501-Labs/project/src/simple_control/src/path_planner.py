#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, PointStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import Int8MultiArray, Int32MultiArray, Int8, Int32
from environment_controller.srv import use_key, use_keyResponse
from Queue import PriorityQueue, Queue
from enum import Enum
from collections import deque

from tf2_geometry_msgs import do_transform_point
import tf2_ros

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
    self.goal = (0,0,3)
    self.got_goal = False

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Set the rate
    self.rate = 40
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
    self.door_sub = rospy.Subscriber('/doors', Int32MultiArray, self.get_doors)
    self.goal_sub = rospy.Subscriber('/goal', Int32, self.get_goal)
    self.dog_sub = rospy.Subscriber("/cell_tower/position", Vector3, self.get_goal)

    # pubs
    self.pos_pub = rospy.Publisher('uav/input/position', Vector3, queue_size=1)
    self.path_pub = rospy.Publisher('/uav/final_path', Int32MultiArray, queue_size=1)
    self.debug_path = rospy.Publisher('/uav/path', Int32MultiArray, queue_size=1)
    # initialize a star planner

    # local queue to get drone to wherever the global queue wants to explore next
    self.local_q = Queue()
    self.global_target = self.start
    self.local_target = self.start

    self.stack = deque()

    self.needToPlanPath = False

    if self.got_goal != (0,0,3):
      self.graph[self.start] = ""
      rospy.loginfo("goal {}".format(self.goal))
      f = (self.goal[0] - self.start[0])**2 + (self.goal[1] - self.start[1])**2
      self.distances[self.start] = 0
      self.global_q.put((0,self.start))
    else:
      rospy.sleep(0.1)

    # Run the node
    self.Run()

  def global_planner_astar(self):
    # rospy.loginfo("pos: "+str(self.position))
    # Check if the current node is the goal

     #  local goal available, move to it

    # rospy.loginfo("printing out self.position: {}".format(self.position))
    # rospy.loginfo("printing out self.local_target: {}".format(self.local_target))
    # rospy.loginfo("printing out the local queue: {}".format(list(self.local_q.queue)))

    if self.local_q.qsize() > 0 and self.position[0:2] == self.local_target[0:2]:
      target = self.local_q.get()
      pub = Vector3()
      pub.x = target[0]
      pub.y = target[1]
      pub.z = target[2]
      self.local_target = target
      rospy.loginfo("local planner next target: {}".format(target))
      self.pos_pub.publish(pub)
      if ( target[2] == 3 ): 
        self.stack.append(target)

    #  reached global+local goal so find neighbors and pick a new global and local goal
    elif self.global_target == self.local_target == self.position:
      
      for direction in [(1,0), (0,1), (-1,0), (0,-1)]:
        # Compute the cost to traverse to that node
        g = self.distances[self.position] + 1
        neighbor = (self.position[0]+direction[0], self.position[1]+direction[1], 3)
        # check if neighboring pixel is empty or open door, and if its in the graph it should be lower cost
        if (neighbor in self.doors and neighbor not in self.graph) or self.map[self.w2g(neighbor)] < 1 and (neighbor not in self.graph or g < self.distances[neighbor]):
          # Compute the estimated cost to goal (heuristic)
          h = (self.goal[0] - neighbor[0])**2 + (self.goal[1] - neighbor[1])**2
          # Add the neighbors to the frontier with the new cost
          f = g + h
          rospy.loginfo("neighbor: {} prio: {}".format(neighbor, f))
          self.graph[neighbor] = self.position
          self.distances[neighbor] = g
          
          if neighbor in self.doors:
            self.global_q = PriorityQueue()
            self.local_q = Queue()
            self.global_q.put((0, neighbor))
            break
          else:
            self.global_q.put((f,neighbor))

      # global_q_items = [str(i) for i in self.global_q.queue if i]
      # global_q_items_str = ' -> '.join(global_q_items)
      # rospy.loginfo("global q: {}".format(global_q_items_str))
      
      # find the best global goal
      dist, node = self.global_q.get()
      self.global_target = node
      rospy.loginfo("A star global planner next target: {}".format(node))
     
      # check if the drone is 1 pixel away from this position
      if abs(self.position[0] - node[0]) + abs(self.position[1] - node[1]) < 2:
        #it is so put in local queue
        self.local_q.put(node)

      else: # handles all backtracking
        # had to back track so find path to the root of the fork
        # path = self.generate_path(self.position, self.graph[node])\
        # rospy.loginfo("node before we call self.graph is: {}".format(node))
        # path = self.generate_path_from_stack(self.position, self.graph[node])
        path = self.prune_path(self.plan(self.position, self.graph[node]))

        for pos in path:
          self.local_q.put(pos)
        self.local_q.put(node)


    else:
      rospy.loginfo("something went wrong, ran out of places to explore but didn't find goal")

  # basically, this state should only enable this node's behavior once per state transition
  # scanning-->hovering
  # and at no time else
  def get_state(self, msg): 
    # if ( DroneState(msg.data) != DroneState.HOVERING and self.state == DroneState.MOVING): 
    #   pass
    # else:   
    #   self.state = DroneState(msg.data)
    #   # rospy.loginfo("path planner node recieved state: {}".format(self.state))

    if ( DroneState(msg.data) != self.state ):
      self.state = DroneState(msg.data)
  
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
      rospy.loginfo("path planner recieved goal: {}, {}".format(goal.point.x+0.5, goal.point.y+0.5))

      self.got_goal = True
      self.goal =(int(round(goal.point.x+0.5)), int(round(goal.point.y+0.5,3)),3 )

  def get_doors(self, msg):
    self.doors = {self.g2w(i) for i in msg.data}
    rospy.loginfo("path planner got door {}".format(self.doors))

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

  def prune_path(self, path): 
    rospy.loginfo("this is the input path: {}".format(path))
    newpath = []  
    newpath.append(path[0])

    for point in range(len(path)): 
      if newpath[-1][0] == path[point][0] or newpath[-1][0] == path[point][1]: 
        pass
      else: 
        newpath.append(path[point - 1])
    newpath.append(path[-1])

    newpath = newpath

    
    rospy.loginfo("this is the pruned path: {}".format(newpath))

    return newpath


    # parent = path[0]
    # path.append((parent[0], parent[1], 4))
    # while parent != end: 
    #   if ( self.stack[-1][0] == path[-1][0] or self.stack[-1][1] == path[-1][1] ):
    #     parent = self.stack.pop()
    #     pass
    #   else:
    #     path.append((parent[0], parent[1], 4))
    #     parent = self.stack.pop()
    #     path.append((parent[0], parent[1], 4))
    
    # path.append( (end[0], end[1], 4) )
    # self.stack.append(end)

    # rospy.loginfo("backtrack path: {}".format(path))

    # return path

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

  def plan(self, drone_position, goal_position):
    rospy.loginfo("this is the start input: {}".format(drone_position))
    rospy.loginfo("this is the end input: {}".format(goal_position))


    # Create both a frontier and a list to track which nodes are already processed
    q = PriorityQueue()
    visited = {}
    # drone_position = (drone_position[0],drone_position[1])
    # goal_position = (goal_position[0],goal_position[1])
    visited[drone_position] = ""
    distances = {}
    # f = abs(goal_position[0] - drone_position[0]) + abs(goal_position[1] - drone_position[1])
    distances[drone_position] = 0
    q.put((0,drone_position))
    # While the frontier is not empty
    while q.qsize() >0:
      rospy.loginfo("this is the q size: {}".format(q.qsize()))
      global_q_items = [str(i) for i in q.queue if i]
      global_q_items_str = ' -> '.join(global_q_items)
      rospy.loginfo("this is the q: {}".format(global_q_items_str))
      
      # Get the node with the lowest cost
      dist, node = q.get()
      rospy.loginfo("printing node in path method: {}".format(node))
      # print(node)
      # Check if the current node is the goal
      if node == goal_position:
        # print("reached goal")
        parent = visited[node]
        path = [node]
        while parent:
          path.append(parent)
          parent = visited[parent]
        path.reverse()
        # print(path)


        return path
      # print(node," is not the goal, ",goal_position, " is")
      # For each neighbor of the current node
      #print(self.get_neighbors(node, map_data))
      for direction in [(1,0), (0,1), (-1,0), (0,-1)]:
        # Compute the cost to traverse to that node
        # g = self.distances[self.position] + 1
        neighbor = (node[0]+direction[0], node[1]+direction[1], 3)
        # Compute the cost to traverse to that node
        g = distances[node] + 1
        
        if (neighbor not in visited or g < distances[neighbor]) and self.map[self.w2g(neighbor)] < 1:
          rospy.loginfo("printing out neighbor from path method: {}".format(neighbor))
          # Compute the estimated cost to goal (heuristic)
          h = abs(goal_position[0] - neighbor[0]) + abs(goal_position[1] - neighbor[1])
          # Add the neighbors to the frontier with the new cost
          f = g + h
          visited[neighbor] = node
          distances[neighbor] = g
          q.put((f,neighbor))
    return None

  def Run(self):
    rate = rospy.Rate(self.rate)
    
    while not rospy.is_shutdown():
      if self.position == self.goal:
          rospy.loginfo("made to goal")
          # self.path = self.generate_path(self.goal, self.start, True)
          # self.path = [self.w2g(i) for i in self.path] don't think we need this line? - Sid
          self.path = []
          for i in self.generate_path(self.goal, self.start, True): 
            self.path.append(i[0])
            self.path.append(i[1])
          
          rospy.loginfo("path: {}".format(self.path))
          self.debug_path.publish(Int32MultiArray(data=self.path))
          rospy.sleep(5)
          rospy.loginfo("data for path: " + str(self.path))
          if ( self.path[-1] != 0 ):  
            self.path_pub.publish(Int32MultiArray(data=self.path))


      if self.state == DroneState.HOVERING:
        self.global_planner_astar()
        self.state = DroneState.MOVING
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