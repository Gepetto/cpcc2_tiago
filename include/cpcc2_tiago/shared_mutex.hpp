#ifndef SHARED_MUTEX_HPP
#define SHARED_MUTEX_HPP

#include <boost/interprocess/sync/named_mutex.hpp>

class SharedMutex {
public:
  SharedMutex() : mutex_(boost::interprocess::open_or_create, "shared_mutex") {}

  void unlock() { mutex_.unlock(); }

  void try_lock() { mutex_.try_lock(); }

  void lock() {
    if (!mutex_.try_lock()) {
      mutex_.lock();
    }
  }

private:
  boost::interprocess::named_mutex mutex_;
};

#endif // SHARED_MUTEX_HPP