#include "voxblox/core/layer.h"
#include <voxblox_msgs/Layer.h>

#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/conversions_inl.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;


using voxblox::EsdfVoxel;
using EsdfLayer = voxblox::Layer<EsdfVoxel>;
using layerMsg = voxblox_msgs::Layer;
// using deserializeMsg = voxblox::deserializeMsgToLayer<EsdfVoxel>;


void esdf_map_message_bind(py::module &m){

    m.def("deserializeMsgToLayer", voxblox::deserializeMsgToLayer<EsdfVoxel>);

    // m.def("deserializeMsgToLayer",
    // [](const layerMsg& layer_msg, EsdfLayer* layer){
    //     // Do some checks

    //     // Run function:
    //     bool success =
    //         voxblox::deserializeMsgToLayer<EsdfVoxel>(layer_msg, layer);

        // if (!success) {
        //     ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
        // } else {
        //     ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
        // }
    // });
}
