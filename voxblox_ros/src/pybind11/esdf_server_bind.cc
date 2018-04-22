
#include "voxblox_ros/esdf_server.h"
#include <ros/ros.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

// using voxblox::EsdfMap;
using voxblox::EsdfServer;
// using voxblox::TsdfServer;
// using EsdfServer = voxblox::EsdfServer;
// using TsdfServer = voxblox::TsdfServer;

// using nh = const ros::NodeHandle;
// using nh_private = const ros::NodeHandle;
// using voxblox::EsdfVoxel;
// using EsdfLayer = voxblox::Layer<EsdfVoxel>;


void esdf_server_bind(py::module &m){

    // py::class_ <TsdfServer >(m, "TsdfServer")
    //     .def(py::init<nh, nh_private>());
    
    py::class_ <EsdfServer >(m, "EsdfServer")
        .def(py::init<const ros::NodeHandle, const ros::NodeHandle>())
        // .def_readwrite("EsdfMap", &EsdfServer::esdf_map_)

        .def("esdfMapCallback", &EsdfServer::esdfMapCallback)

        .def("getEsdfDistanceAtPosition", &EsdfServer::getDistanceAtPosition);
        // .def("getBatchEsdfDistanceAtPosition", &EsdfServer::getBatchDistanceAtPosition);

}


// Specific accessor functions for esdf maps.
// Returns true if the point exists in the map AND is observed.
// These accessors use Vector3d and doubles explicitly rather than
// FloatingPoint to have a standard, cast-free interface to planning
// functions.
//   bool getDistanceAtPosition(const Eigen::Vector3d& position,
//  double* distance) const;