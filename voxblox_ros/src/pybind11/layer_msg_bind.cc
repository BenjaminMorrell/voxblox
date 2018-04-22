#include "voxblox_msgs/Layer.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

using Layer = voxblox_msgs::Layer;

void layer_msg_bind(py::module &m) {

  py::class_<Layer>(m, "Layer")
      .def_readwrite("voxel_size", &Layer::voxel_size)
      .def_readwrite("voxels_per_side", &Layer::voxels_per_side)
      .def_readwrite("layer_type", &Layer::layer_type)
      .def_readwrite("action", &Layer::action)
      .def_readwrite("blocks", &Layer::blocks);
}
