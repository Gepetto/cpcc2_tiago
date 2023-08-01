#include "cpcc2_tiago/parallel_croc_solver.hpp"

void read_params() {
  n_joints_ = params_.joints.size();
  joints_names_.reserve(n_joints_);
  joints_names_ = params_.joints;
}

void resize_vectors() {
  x_meas_.resize(2 * n_joints_);
  xs_.resize(2 * n_joints_);
  us_.resize(n_joints_);
  Ks_.resize(n_joints_, 2 * n_joints_);
}

void init_shared_memory() {
  // x_meas_shm_ = boost::interprocess::shared_memory_object(
  //     boost::interprocess::open_only, "x_meas_shm",
  //     boost::interprocess::read_write);

  // x_meas_region_ = boost::interprocess::mapped_region(
  //     x_meas_shm_, boost::interprocess::read_write);

  // x_meas_data_ptr_ = static_cast<double *>(x_meas_region_.get_address());

  // x_meas_smh_vec_.resize(x_meas_.size());

  // x_meas_smh_vec_ =
  //     Eigen::Map<Eigen::VectorXd>(x_meas_data_ptr_, x_meas_.size());

  //   us_shm_ = boost::interprocess::shared_memory_object(
  //       boost::interprocess::open_only, "us_shm",
  //       boost::interprocess::read_write);

  //   us_region_ = boost::interprocess::mapped_region(
  //       us_shm_, boost::interprocess::read_write);

  //   us_smh_ptr_ = static_cast<Eigen::VectorXd *>(us_region_.get_address());

  //   xs_shm_ = boost::interprocess::shared_memory_object(
  //       boost::interprocess::open_only, "xs_shm",
  //       boost::interprocess::read_write);

  //   xs_region_ = boost::interprocess::mapped_region(
  //       xs_shm_, boost::interprocess::read_write);

  //   xs_smh_ptr_ = static_cast<Eigen::VectorXd *>(xs_region_.get_address());

  //   Ks_shm_ = boost::interprocess::shared_memory_object(
  //       boost::interprocess::open_only, "K_shm",
  //       boost::interprocess::read_write);

  //   Ks_region_ = boost::interprocess::mapped_region(
  //       Ks_shm_, boost::interprocess::read_write);

  //   Ks_smh_ptr_ = static_cast<Eigen::MatrixXd *>(Ks_region_.get_address());

  //   target_shm_ = boost::interprocess::shared_memory_object(
  //       boost::interprocess::open_only, "target_shm",
  //       boost::interprocess::read_write);

  //   target_region_ = boost::interprocess::mapped_region(
  //       target_shm_, boost::interprocess::read_write);

  //   target_smh_ptr_ =
  //       static_cast<Eigen::Vector3d *>(target_region_.get_address());
}

int main() {
  read_params();
  init_shared_memory();

  while (true) {
    if (mutex2_.try_lock()) {
      break;
    }

    if (!mutex2_.timed_lock(boost::get_system_time() +
                            boost::posix_time::milliseconds(10))) {
      mutex2_.unlock();
    }
  }

  Eigen::VectorXd vec;
  vec.resize(x_meas_.size());

  mutex2_.lock();

  using namespace boost::interprocess;

  using ShmemAllocator =
      allocator<double, managed_shared_memory::segment_manager>;
  using MyVector = vector<double, ShmemAllocator>;

  managed_shared_memory x_meas_shm_(open_only, "shared_memory");
  MyVector *myVector = x_meas_shm_.find<MyVector>("MyVector").first;

  std::cout << "Size of shared vector: " << myVector->size() << std::endl;

  mutex2_.unlock();
  // #include <stdlib.h>

  while (true) {
    sleep(1);
    mutex2_.lock();
    //     vec = x_meas_smh_vec_;
    MyVector::iterator iter = myVector->begin();
    while (iter != myVector->end()) {
      std::cout << " " << *iter << " ";
      iter++;
    }

    std::cout << std::endl;
    mutex2_.unlock();
    //     std::cout << vec.transpose() << std::endl;
  }
}