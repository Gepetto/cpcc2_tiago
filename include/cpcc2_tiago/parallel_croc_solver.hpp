#ifndef PARALLEL_CROC_SOLVER_HPP
#define PARALLEL_CROC_SOLVER_HPP

#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/thread/thread_time.hpp>
#include <iostream>

#include "cpcc2_tiago/model_builder.hpp"
#include "cpcc2_tiago/shared_mutex.hpp"
#include "cpcc2_tiago/tiago_OCP_maker.hpp"

// auto-generated by generate_parameter_library
#include "cpcc2_tiago_parameters.hpp"

cpcc2_tiago::Params params_; // load parmeters from yaml file

boost::interprocess::named_mutex mutex_{boost::interprocess::open_or_create,
                                        "mutex"};

typedef boost::interprocess::allocator<
    double, boost::interprocess::managed_shared_memory::segment_manager>
    shm_allocator;

// Alias a vector that uses the previous STL-like allocator
typedef boost::interprocess::vector<double, shm_allocator> shared_vector;

boost::interprocess::managed_shared_memory crocoddyl_shm_;

Model model_;
Data data_;

FrameIndex lh_id_;

tiago_OCP::OCP OCP_tiago_;

int OCP_horizon_length_;
double OCP_time_step_;

std::vector<std::string> joints_names_;
int n_joints_;

Eigen::VectorXd x_meas_;
shared_vector *x_meas_shm_;

Eigen::VectorXd us_;
shared_vector *us_shm_;

Eigen::VectorXd xs_;
shared_vector *xs_shm_;

Eigen::MatrixXd Ks_;
shared_vector *Ks_shm_;

Eigen::Vector3d target_;
shared_vector *target_smh_;

bool *solver_started_shm_;
bool *is_first_update_done_shm_;
bool is_first_update_done_ = false;

bool solved_first_ = false;

void read_params();
void resize_vectors();

void initilize_shared_vectors();

Eigen::VectorXd read_controller_x();
Eigen::Vector3d read_controller_target();
void send_controller_result(Eigen::VectorXd us, Eigen::VectorXd xs,
                            Eigen::MatrixXd Ks);

void init_shared_memory();

#endif // PARALLEL_CROC_SOLVER_HPP