import rospy

import voxblox
import voxblox_ros
from voxblox_msgs.msg import Layer
import numpy as np

import struct

metres_to_ntsdf = 1.0
max_ntsdf_voxel_weight = 1.0

def unpack_message(block):

  

  count = 0
  for byte in data:

    if np.mod(count,2) == 0: # if even
      # Take out the bitshift to get 
      import pdb; pdb.set_trace()

      distance = (byte >> 16) / metres_to_ntsdf
      weight = (byte & 0x0000FFFF)*max_ntsdf_voxel_weight/0xFFFF


      count  = count + 1
      # b1 = (byte >> 16) / metres_to_ntsdf #struct.unpack('H',byte >> 16)[0]/metres_to_ntsdf
      # b2 = struct.unpack('H', byte & 0x0000FFFF)[0]*max_ntsdf_voxel_weight/0xFFFF
      # 0xFFFF - uint16 max (I think...)
      # try 
      # x = struct.unpack("H",struct.pack("H",struct.unpack("I",struct.pack("I",byte))[0]>>16))[0]
      # or just
      # x = byte >> 16

      # x2 = struct.unpack("H",struct.pack("H",struct.unpack("I",struct.pack("I",byte))[0] & 0x0000FFFF))[0]
      # # or just
      # x2 = byte & 0x0000FFFF



def callback(data):

  rospy.loginfo(rospy.get_caller_id() + "In callback from esdf listening in python")

  # Try to pull out parts of the message
  rospy.loginfo("Voxel size is: %f, and there are %d voxels per size", data.voxel_size,data.voxels_per_side)

  rospy.loginfo("Layer type is: %s",data.layer_type)

  rospy.loginfo("action is: %d",data.action)

  rospy.loginfo("Number of blocks: %d", len(data.blocks))

  esdf_layer = voxblox.EsdfLayer(data.voxel_size,data.voxels_per_side)

  # layermsg = voxblox_ros.Layer
  # layermsg = Layer
  # import pdb; pdb.set_trace()

  # Loop through each block in the message
  for block_msg in data.blocks:
    # Get the block from the index (function will vreate a new one if it doesn't exist yet)
    block = esdf_layer.allocateBlockPtrByIndex(np.array([block_msg.x_index, block_msg.y_index, block_msg.z_index], dtype='int'))

    # block.deserializeFromIntegers(block_msg.data)
    block.deserializeFromIntegers(np.array(block_msg.data,dtype="uint32").tolist())

    # # Loop through the data in the block
    # voxel_id = 0
    # count = 0

  #   for packet in block_msg.data: # range(0,len(block.data)): # For each data packet
      
  #     if np.mod(count,2) == 0: # if even
  #       # Get the current voxel
  #       voxel = block.getVoxelByLinearIndex(voxel_id)
  #       voxel.distance = packet
  #       # voxel.distance[0:0+4] = packet # try something like memcpy? https://stackoverflow.com/questions/13689628/is-there-a-python-equivalent-to-memcpy 
  #       # voxel.weight = (packet & 0x0000FFFF)*max_ntsdf_voxel_weight/0xFFFF
  #       # print("Voxel {} has distance {}".format(voxel_id,voxel.distance))
  #       voxel_id = voxel_id + 1
  #     else:
  #       pass # Second byte is color - don't care about this
  #     count = count + 1
  # import pdb; pdb.set_trace()
  
  esdf_layer.saveToFile("/home/bjm/TORQ/gcs_ws/src/torq/torq_gcs/mesh/test_unreal_esdf_new.proto")

  # import pdb; pdb.set_trace()
  # import pdb; pdb.set_trace()
  # b1 = esdf_layer.allocateBlockPtrByIndex(np.array([0, 0, 0], dtype='int'))

  # b1.deserializeFromIntegers(data.blocks[0].data)

  # esdf_layer.saveToFile("/home/bjm/TORQ/gcs_ws/src/torq/torq_gcs/mesh/test_unreal_esdf2.proto")
  # b1.deserializeFromIntegers(np.array(data.blocks[0].data,dtype="int"))

  # v1 = b1.getVoxelByCoordinates(np.array([0, 0, 0.5], dtype='double'))
  # print(v1.distance)

  # result = voxblox_ros.deserializeMsgToLayer(layermsg,esdf_layer)

  # print(type(data.blocks[0]))
  # print(dir(data.blocks[0]))

  # voxblox.EsdfMap
  # self.esdf_layer = voxblox.loadEsdfLayer(filename)
  # self.global_dict['fsp_out_map'] = voxblox.EsdfMap(self.esdf_layer)
  # import pdb; pdb.set_trace()
  # unpack_message(data.blocks[0].data)

# def readEsdfLayerMessage(layer_msg):

  # Checks on the message


  # # Loop through each block in the message
  # for block_msg in layer_msg.blocks:
  #   # Get the block from the index (function will vreate a new one if it doesn't exist yet)
  #   block = esdf_layer.allocateBlockPtrByIndex(np.array([block_msg.x_index, block_msg.y_index, block_msg.z_index], dtype='int'))

  #   # Loop through the data in the block
  #   voxel_id = 0
  #   count = 0
  #   for packet in block_msg.data: # range(0,len(block.data)): # For each data packet
      
  #     if np.mod(count,2) == 0: # if even
  #       # Get the current voxel
  #       voxel = block.getVoxelByLinearIndex(voxel_id)
  #       voxel.distance = (packet >> 16) / metres_to_ntsdf
  #       # voxel.weight = (packet & 0x0000FFFF)*max_ntsdf_voxel_weight/0xFFFF
  #       print("Voxel {} has distance {}".format(voxel_id,voxel.distance))
  #       voxel_id = voxel_id + 1
  #     else:
  #       pass # Second byte is color - don't care about this
  #     count = count + 1

  # esdf_layer.saveToFile("/home/bjm/TORQ/gcs_ws/src/torq/torq_gcs/mesh/test_unreal_esdf.proto")



def listener():

  rospy.init_node('esdf_listenerpy',anonymous=True)

  rospy.Subscriber("/esdf_server/esdf_map_out",Layer,callback)
  
  # esdfServer = voxblox_ros.EsdfServer()

  rospy.spin()

if __name__ == '__main__':
  listener()