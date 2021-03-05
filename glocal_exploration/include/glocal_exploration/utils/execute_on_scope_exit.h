#ifndef GLOCAL_EXPLORATION_UTILS_EXECUTE_ON_SCOPE_EXIT_H_
#define GLOCAL_EXPLORATION_UTILS_EXECUTE_ON_SCOPE_EXIT_H_

#include <functional>

namespace glocal_exploration {

class ExecuteOnScopeExit {
 public:
  using Function = std::function<void()>;

  // Prevent copying
  ExecuteOnScopeExit(const ExecuteOnScopeExit&) = delete;
  ExecuteOnScopeExit& operator=(const ExecuteOnScopeExit&) = delete;

  explicit ExecuteOnScopeExit(const Function& f) : function_(f) {}
  ~ExecuteOnScopeExit() { function_(); }

 private:
  Function function_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_UTILS_EXECUTE_ON_SCOPE_EXIT_H_
