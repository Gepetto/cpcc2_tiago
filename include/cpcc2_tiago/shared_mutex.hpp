#ifndef SHARED_MUTEX_HPP
#define SHARED_MUTEX_HPP

#include <boost/interprocess/sync/named_mutex.hpp>

class SharedMutex {
 public:
  SharedMutex() : mutex_(boost::interprocess::open_or_create, "shared_mutex") {}

  void try_lock() { mutex_.try_lock(); }

  void lock() { mutex_.lock(); }

  void unlock() { mutex_.unlock(); }

 private:
  boost::interprocess::named_mutex mutex_;
};

#endif  // SHARED_MUTEX_HPP