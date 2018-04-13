import rospy

import voxblox
import voxblox_ros
from voxblox_msgs.msg import Layer


def callback(data):

  rospy.loginfo(rospy.get_caller_id() + "In callback from esdf listening in python")

  # Try to pull out parts of the message
  rospy.loginfo("Voxel size is: %f, and there are %d voxels per size", data.voxel_size,data.voxels_per_side)

  rospy.loginfo("Layer type is: %s",data.layer_type)

  rospy.loginfo("action is: %d",data.action)

  rospy.loginfo("Number of blocks: %d", len(data.blocks))

  # esdf_layer = voxblox.EsdfLayer(data.voxel_size,
  #                                               data.voxels_per_side)


  


  # voxblox.EsdfMap
  # self.esdf_layer = voxblox.loadEsdfLayer(filename)
  # self.global_dict['fsp_out_map'] = voxblox.EsdfMap(self.esdf_layer)

def listener():

  rospy.init_node('esdf_listenerpy',anonymous=True)

  rospy.Subscriber("/esdf_server/esdf_map_out",Layer,callback)
  
  # esdfServer = voxblox.EsdfServer()

  rospy.spin()

if __name__ == '__main__':
  listener()