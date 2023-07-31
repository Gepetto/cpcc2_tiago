#ifndef PARALLEL_CROC_SOLVER_HPP
#define PARALLEL_CROC_SOLVER_HPP

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <iostream>

#include "cpcc2_tiago/model_builder.hpp"
#include "cpcc2_tiago/shared_mutex.hpp"
#include "cpcc2_tiago/tiago_OCP_maker.hpp"

SharedMutex mutex_;

boost::interprocess::shared_memory_object joints_shm_;
boost::interprocess::mapped_region joints_region_;
std::vector<std::string> *joints_smh_ptr_;

boost::interprocess::shared_memory_object x_meas_shm_;
boost::interprocess::mapped_region x_meas_region_;

boost::interprocess::shared_memory_object us_shm_;
boost::interprocess::mapped_region us_region_;

boost::interprocess::shared_memory_object xs_shm_;
boost::interprocess::mapped_region xs_region_;

boost::interprocess::shared_memory_object Ks_shm_;
boost::interprocess::mapped_region Ks_region_;

boost::interprocess::shared_memory_object target_shm_;
boost::interprocess::mapped_region target_region_;

Eigen::VectorXd x_meas_;
Eigen::VectorXd *x_meas_smh_ptr_;

Eigen::VectorXd us_;
Eigen::VectorXd *us_smh_ptr_;

Eigen::VectorXd xs_;
Eigen::VectorXd *xs_smh_ptr_;

Eigen::MatrixXd Ks_;
Eigen::MatrixXd *Ks_smh_ptr_;

Eigen::Vector3d *target_smh_ptr_;

Eigen::Vector3d pos_error_;

Eigen::Vector3d end_effector_pos_;

void init_shared_memory();

#endif // PARALLEL_CROC_SOLVER_HPP