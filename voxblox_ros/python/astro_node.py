import rospy
import sys, os
from python_qt_binding.QtWidgets import QApplication
from minsnap import *

import pickle

import voxblox
import voxblox_ros
from voxblox_msgs.msg import Layer
import numpy as np




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

    rospy.loginfo(rospy.get_caller_id() + "In callback from esdf listening in python")

    # Try to pull out parts of the message
    rospy.loginfo("Voxel size is: %f, and there are %d voxels per size", msg.voxel_size,msg.voxels_per_side)

    rospy.loginfo("Layer type is: %s",msg.layer_type)

    rospy.loginfo("action is: %d",msg.action)

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
    print("\n\nRunning ASTRO\n\n")
    self.updateEsdfObstacle()
    self.planTrajectory()
    print("Saving Trajectory...")
    self.saveTrajectory()
    print("Saved trajectory.")


if __name__ == '__main__':

  # Start node
  rospy.init_node('astro',anonymous=True)

  # Init class
  plan = Planner()

  # Set up start and goal 
  plan.start['x'] = 9.0
  plan.start['y'] = 3.5
  plan.start['z'] = 3.5
  plan.start['yaw'] = 0.0
  plan.goal['x'] = 9.0
  plan.goal['y'] = -5.0
  plan.goal['z'] = 4.5
  plan.goal['yaw'] = 0.0

  plan.initialisePlanner()

  # Example use case:
  plan.updateWaypoints(plan.start,plan.goal)

  # Create Subscriber
  rospy.Subscriber("/esdf_server/esdf_map_out",Layer,plan.readESDFMapMessage)

  # Spin
  rospy.spin()
  



  

  
