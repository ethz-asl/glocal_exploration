#ifndef GLOCAL_EXPLORATION_UTILITY_CONFIG_CHECKER_H_
#define GLOCAL_EXPLORATION_UTILITY_CONFIG_CHECKER_H_

#include <string>
#include <utility>

#include <glog/logging.h>

namespace glocal_exploration {

/**
 * Utility tool to make checking configs easier and more readable.
 */
class ConfigChecker {
 public:
  explicit ConfigChecker(std::string module_name)
      : name_(std::move(module_name)), is_valid_(true) {}
  ~ConfigChecker() = default;

  [[nodiscard]] bool isValid() const { return is_valid_; }

  template <typename T>
  void check_gt(const T& param, const T& value, const std::string& name) {
    if (param <= value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected > "
                   << value << " (is: " << param << ").";
      is_valid_ = false;
    }
  }

  template <typename T>
  void check_ge(const T& param, const T& value, const std::string& name) {
    if (param < value) {
      LOG(WARNING) << name_ << ": param '" << name
                   << "' is expected >= " << value << " (is: " << param << ").";
      is_valid_ = false;
    }
  }

  template <typename T>
  void check_lt(const T& param, const T& value, const std::string& name) {
    if (param >= value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected < "
                   << value << " (is: " << param << ").";
      is_valid_ = false;
    }
  }

  template <typename T>
  void check_le(const T& param, const T& value, const std::string& name) {
    if (param > value) {
      LOG(WARNING) << name_ << ": param '" << name
                   << "' is expected <= " << value << " (is: " << param << ").";
      is_valid_ = false;
    }
  }

  template <typename T>
  void check_eq(const T& param, const T& value, const std::string& name) {
    if (param != value) {
      LOG(WARNING) << name_ << ": param '" << name << "' is expected to be "
                   << value << " (is: " << param << ").";
      is_valid_ = false;
    }
  }

  void check(bool param, const std::string& warning) {
    if (!param) {
      LOG(WARNING) << name_ << ": " << warning;
      is_valid_ = false;
    }
  }

 private:
  const std::string name_;
  bool is_valid_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_UTILITY_CONFIG_CHECKER_H_
