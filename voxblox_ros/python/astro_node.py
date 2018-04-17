import rospy
import sys, os
from python_qt_binding.QtWidgets import QApplication
# from minsnap import *

import pickle

import voxblox
import voxblox_ros
from voxblox_msgs.msg import Layer
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import torq_gcs
from torq_gcs.plan.astro_plan import QRPolyTrajGUI
# import diffeo



# import struct
class Planner:

  def __init__(self):
    

    # Global goal variable
    self.goal = dict()
    # Global start variable
    self.start = dict()

    self.app = QApplication( sys.argv )

    self.global_dict = dict()

    # Time for publishing
    self.setpointCount = 0
    self.time = 0.0
    self.tmax = 100.0

    # Times for replanning
    self.replanHz = 0.1
    self.timeOfReplan = 0.0
    self.startDeltaT = 0.5 # Time ahead of current time to use as the start location
    self.firstPlan = True

    # Initialise the map
    self.esdfLayer = voxblox.EsdfLayer(0.2,16) # Start with default values
    self.global_dict['fsp_out_map'] = voxblox.EsdfMap(self.esdfLayer)# simple implementation now

    # Init planner GCS object
    self.planner = torq_gcs.plan.astro_plan.QRPolyTrajGUI(self.global_dict,defer=True,curv_func=False)

  def initialisePlanner(self):
    # Waypoints
    waypoints = dict()
    for key in self.start.keys():
      waypoints[key] = np.zeros([1,2])
      waypoints[key][0,0] = self.start[key]
      waypoints[key][0,1] = self.goal[key]

    # Set time to complete the trajectory
    self.planner.seed_times = np.array([self.tmax])
    
    self.global_dict['disc_out_waypoints'] = waypoints

    # Initial guess to load waypoints and initialise planner
    self.planner.on_disc_updated_signal()

  def updateStart(self,position):
    # position is a dict with keys x, y, z
    self.planner.on_waypoint_control_callback(position, 0, "main")

  def updateGoal(self,position):
    # position is a dict with keys x, y, z
    self.planner.on_waypoint_control_callback(position, np.shape(self.planner.qr_polytraj.waypoints['x'])[1]-1,"main")

  def updateWaypoints(self,start, goal):

    self.updateStart(start)
    self.updateGoal(goal)

  def planTrajectory(self):
    # Runs the optimisation and updates the trajectory 
    self.planner.on_run_astro_button_click()

    self.planner.qr_polytraj.exit_on_feasible = True
    self.planner.qr_polytraj.optimise(mutate_serial=3)
    self.planner.qr_polytraj.get_trajectory()
    self.planner.update_path_markers()
    
  def updateEsdfObstacle(self):
    # TODO Some check on if the ESDF Map contains anything
    if not ('fsp_out_map' in self.global_dict):
      print("No ESDF loaded to use in planner. Not planning")
      return

    # Remove ESDF constraint if there are any already
    self.planner.qr_polytraj.remove_esdf_constraint()

    # Add ESDF constraint
    self.planner.load_esdf_obstacle()

  def saveTrajectory(self,filename="/home/bjm/TORQ/gcs_ws/src/torq/torq_gcs/waypoints/test_traj.traj"):

    qrp_out = self.planner.qr_polytraj

    with open(filename, 'wb') as f:
      # Remove ESDF constraint if there are any already
      qrp_out.remove_esdf_constraint()

      pickle.dump(qrp_out, f, 2 )

  def readESDFMapMessage(self,msg):

    # rospy.loginfo(rospy.get_caller_id() + "In callback from esdf listening in python")

    # Try to pull out parts of the message
    rospy.loginfo("Voxel size is: %f, and there are %d voxels per side", msg.voxel_size,msg.voxels_per_side)

    # rospy.loginfo("Layer type is: %s",msg.layer_type)

    # rospy.loginfo("Action is: %d",msg.action)

    rospy.loginfo("Number of blocks: %d", len(msg.blocks))

    esdfLayer = voxblox.EsdfLayer(msg.voxel_size,msg.voxels_per_side)

    # Loop through each block in the message
    for block_msg in msg.blocks:
      # Get the block from the index (function will vreate a new one if it doesn't exist yet)
      block = esdfLayer.allocateBlockPtrByIndex(np.array([block_msg.x_index, block_msg.y_index, block_msg.z_index], dtype='int'))

      # block.deserializeFromIntegers(block_msg.data)
      block.deserializeFromIntegers(np.array(block_msg.data,dtype="uint32").tolist())

    # update map TODO(TDBM) can this process be made more efficient to udpate the representation?
    esdfMap = voxblox.EsdfMap(esdfLayer)

    self.global_dict['fsp_out_map'] = esdfMap

    # Run planner
    if self.time > 1/self.replanHz or self.firstPlan: # If the time since the last replan is more than the desired period
      self.resetStartFromTraj()
      print("\n\nTime to replan ({}): Running ASTRO\n\n".format(self.time))
      self.time = 0.0 # Starting at the start of the new trajectory
      self.updateEsdfObstacle()
      self.planTrajectory()
      print("\n\n\t\t COMPLETED TRAJECTORY PLAN \n\n")
      
      # Reset times:
      # self.timeOfReplan = self.time
      

      self.firstPlan = False
      # self.saveTrajectory()
      

  def getSetpointAtTime(self):

    # Get the state at the current time being tracked
    output = self.planner.on_animate_eval_callback(self.time)
    # output format is: (t_max, x, y, z, q[0], q[1], q[2], q[3])

    # Set tmax to match 
    self.tmax = output[0]

    if self.time > self.tmax:
      self.time = self.tmax

    # Fill message
    msg = PoseStamped()

    # Header
    msg.header.seq = self.setpointCount
    self.setpointCount = self.setpointCount+1 # increment count
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "local_origin"

    # Position
    msg.pose.position.x = output[1]
    msg.pose.position.y = output[2]
    msg.pose.position.z = output[3]

    # Orientation
    msg.pose.orientation.w = output[4]
    msg.pose.orientation.x = output[5]
    msg.pose.orientation.y = output[6]
    msg.pose.orientation.z = output[7]
    
    # Publish message
    # print("Computed setpoint is:\n{}".format(msg))

    return msg

  def resetStartFromTraj(self):
    # Resets the start location and the planned time

    # Time to start replan
    startTime = self.time + self.startDeltaT
    
    # State for the start
    self.start = self.planner.get_state_at_time(startTime)
    # output = self.planner.on_animate_eval_callback(startTime)
    # TODO Get out the derivatives as well - to set the new location to be continuous

    # Change the start state
    # self.start['x'] = output[1]
    # self.start['y'] = output[2]
    # self.start['z'] = output[3]
    # self.start['yaw'] = 0.0 # TODO convert from the quaterion to get the yaw
    self.updateStart(self.start)
    # Reset planned trajectory time
    self.planner.qr_polytraj.update_times([0],self.tmax-startTime,defer=True)

    self.tmax -= startTime

    print("\n\nRESET: New duration is {}\nStart Location is: {}".format(self.tmax,self.start))

if __name__ == '__main__':

  # Start node
  rospy.init_node('astro',anonymous=True)

  # Init class
  plan = Planner()

  # Set up start and goal 
  plan.start['x'] = 5.0
  plan.start['y'] = 0.9
  plan.start['z'] = 4.5
  plan.start['yaw'] = 0.0
  plan.goal['x'] = 12.0
  plan.goal['y'] = 0.9
  plan.goal['z'] = 4.5
  plan.goal['yaw'] = 0.0

  plan.initialisePlanner()

  # Example use case:
  plan.updateWaypoints(plan.start,plan.goal)

  # Create Subscriber
  rospy.Subscriber("/esdf_server/esdf_map_out",Layer,plan.readESDFMapMessage)

  # pub = rospy.Publisher("topic",String,queue_size=1)
  setpoint_pub = rospy.Publisher("setpoint",PoseStamped,queue_size=1)

  # Spin
  # rospy.spin()

  rateHz = 5.0

  r = rospy.Rate(rateHz) # 10hz
  while not rospy.is_shutdown():
      # pub.publish("hello")
      msg = plan.getSetpointAtTime()
      setpoint_pub.publish(msg)
      # increment time
      plan.time += 1.0/rateHz # TODO WARNING - this is not going to accurately track time
      r.sleep()
      
      


  

  
