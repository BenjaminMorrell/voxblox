#include <pybind11/pybind11.h>
namespace py = pybind11;

void esdf_server_bind(py::module &);

PYBIND11_MODULE(voxblox_rospy, m) {
  esdf_server_bind(m);
}
