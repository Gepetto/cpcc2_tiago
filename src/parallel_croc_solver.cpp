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

  crocoddyl_shm_ = boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only,
      "crcoddyl_shm"); // segment name

  // Find the vector using the c-string name
  x_meas_shm_ = crocoddyl_shm_.find<shared_vector>("x_meas_shm").first;
  us_shm_ = crocoddyl_shm_.find<shared_vector>("us_shm").first;
  xs_shm_ = crocoddyl_shm_.find<shared_vector>("xs_shm").first;
  Ks_shm_ = crocoddyl_shm_.find<shared_vector>("Ks_shm").first;
  target_smh_ = crocoddyl_shm_.find<shared_vector>("target_shm").first;
}

Eigen::VectorXd read_controller_x() {
  mutex_.lock();
  Eigen::VectorXd x =
      Eigen::Map<Eigen::VectorXd>(x_meas_shm_->data(), x_meas_shm_->size());
  mutex_.unlock();
  return x;
}

void send_controller_result(Eigen::VectorXd us, Eigen::VectorXd xs,
                            Eigen::MatrixXd Ks) {
  mutex_.lock();
  us_shm_->assign(us.data(), us.data() + us.size());
  xs_shm_->assign(xs.data(), xs.data() + us.size());
  Ks_shm_->assign(Ks.data(), Ks.data() + us.size());
  mutex_.unlock();
}

int main() {
  read_params();

  while (true) {
    if (mutex_.try_lock()) {
      break;
    }

    if (!mutex_.timed_lock(boost::get_system_time() +
                           boost::posix_time::milliseconds(10))) {
      mutex_.unlock();
    }
  }

  mutex_.unlock();

  sleep(1);

  init_shared_memory();

  while (true) {
    sleep(1);
    x_meas_ = read_controller_x();

    std::cout << x_meas_.transpose() << std::endl;

    //     std::cout << vec.transpose() << std::endl;
  }
}