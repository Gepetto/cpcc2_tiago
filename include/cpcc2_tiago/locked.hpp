#pragma once

// STL
#include <functional>
#include <mutex>

namespace cpcc2_tiago {

template <typename T>
class Unlocked;

template <typename T>
class Locked {
 public:
  friend Unlocked<T>;

  Locked() = default;
  template <typename... Args>
  Locked(Args &&...args) : T{std::forward<Args...>(args...)} {}
  Locked(const Locked &) = delete;
  Locked(Locked &&) = delete;

  ~Locked() = default;

  template <typename Arg>
  Locked &operator=(Arg &&arg) {
    std::lock_guard lock(mutex_);
    data_ = std::forward<Arg>(arg);
    return *this;
  }
  Locked &operator=(const Locked &) = delete;
  Locked &operator=(Locked &&) = delete;

  template <typename Function>
  inline void operator()(Function func) {
    std::lock_guard lock(mutex_);
    func(std::ref(data_));
  }
  template <typename Function>
  inline void operator()(Function func) const {
    std::lock_guard lock(mutex_);
    func(std::cref(data_));
  }

  inline operator T() {
    std::lock_guard lock(mutex_);
    T copy = data_;
    return copy;
  }
  inline operator T() const {
    std::lock_guard lock(mutex_);
    T copy = data_;
    return copy;
  }

  inline Unlocked<T> unlock() { return Unlocked<T>{*this}; }

 protected:
  T data_;
  mutable std::mutex mutex_;
};

template <typename T>
class Unlocked {
 public:
  Unlocked() = default;
  template <typename... Args>
  Unlocked(Locked<T> &locked) : locked_{locked}, lock_{locked_.mutex_} {}
  Unlocked(const Unlocked &) = delete;
  Unlocked(Unlocked &&) = default;

  ~Unlocked() = default;

  template <typename Arg>
  Unlocked &operator=(Arg &&arg) {
    locked_.data_ = std::forward<Arg>(arg);
    return *this;
  }
  Unlocked &operator=(const Unlocked &) = delete;
  Unlocked &operator=(Unlocked &&) = default;

  T *operator->() { return &locked_.data_; }
  T const *operator->() const { return &locked_.data_; }

  inline operator T() {
    T copy = locked_.data_;
    return copy;
  }
  inline operator T() const {
    T copy = locked_.data_;
    return copy;
  }

 private:
  Locked<T> &locked_;
  std::unique_lock<std::mutex> lock_;
};

}  // namespace cpcc2_tiago
