#include <pybind11/pybind11.h>
namespace py = pybind11;

// void layer_bind(py::module &);
// void esdf_server_bind(py::module &);
void esdf_map_message_bind(py::module &);
void layer_msg_bind(py::module &);

PYBIND11_MODULE(voxblox_rospy, m) {
  // layer_bind(m);
  // esdf_server_bind(m);
  esdf_map_message_bind(m);
  layer_msg_bind(m);
}
