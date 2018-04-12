import rospy

import voxblox
from voxblox_msgs.msg import Layer


def callback(data):

  rospy.loginfo(rospy.get_caller_id() + "In callback from esdf listening in python")

  # voxblox.EsdfMap
  # self.esdf_layer = voxblox.loadEsdfLayer(filename)
  # self.global_dict['fsp_out_map'] = voxblox.EsdfMap(self.esdf_layer)

def listener():

  rospy.init_node('esdf_listenerpy',anonymous=True)

  rospy.Subscriber("/esdf_server/esdf_map_out",Layer,callback)

  rospy.spin()

if __name__ == '__main__':
  listener()