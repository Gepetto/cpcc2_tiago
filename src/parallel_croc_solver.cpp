#include "cpcc2_tiago/parallel_croc_solver.hpp"

void init_shared_memory() {

  joints_shm_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_or_create, "joints_shm",
      boost::interprocess::read_write);

  joints_region_ = boost::interprocess::mapped_region(
      joints_shm_, boost::interprocess::read_write);

  joints_smh_ptr_ =
      static_cast<std::vector<std::string> *>(joints_region_.get_address());

  x_meas_shm_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_only, "x_meas_shm",
      boost::interprocess::read_write);

  x_meas_region_ = boost::interprocess::mapped_region(
      x_meas_shm_, boost::interprocess::read_write);

  x_meas_smh_ptr_ =
      static_cast<Eigen::VectorXd *>(x_meas_region_.get_address());

  us_shm_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_only, "us_shm",
      boost::interprocess::read_write);

  us_region_ = boost::interprocess::mapped_region(
      us_shm_, boost::interprocess::read_write);

  us_smh_ptr_ = static_cast<Eigen::VectorXd *>(us_region_.get_address());

  xs_shm_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_only, "xs_shm",
      boost::interprocess::read_write);

  xs_region_ = boost::interprocess::mapped_region(
      xs_shm_, boost::interprocess::read_write);

  xs_smh_ptr_ = static_cast<Eigen::VectorXd *>(xs_region_.get_address());

  Ks_shm_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_only, "K_shm", boost::interprocess::read_write);

  Ks_region_ = boost::interprocess::mapped_region(
      Ks_shm_, boost::interprocess::read_write);

  Ks_smh_ptr_ = static_cast<Eigen::MatrixXd *>(Ks_region_.get_address());

  target_shm_ = boost::interprocess::shared_memory_object(
      boost::interprocess::open_only, "target_shm",
      boost::interprocess::read_write);

  target_region_ = boost::interprocess::mapped_region(
      target_shm_, boost::interprocess::read_write);

  target_smh_ptr_ =
      static_cast<Eigen::Vector3d *>(target_region_.get_address());
}

int main() {
  init_shared_memory();
  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!" << std::endl;
  // mutex_.lock();
  //  std::cout << (*joints_smh_ptr_)[0] << std::endl;
  //  mutex_.unlock();
}