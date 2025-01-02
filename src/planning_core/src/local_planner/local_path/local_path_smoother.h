#ifndef LOCAL_PATH_SMOOTHER_H_
#define LOCAL_PATH_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning {
  class LocalPathSmoother
  {
  public:
    LocalPathSmoother();
    LocalPathSmoother(const LocalPathSmoother&) = delete;
    LocalPathSmoother operator=(const LocalPathSmoother&) = delete;
    ~LocalPathSmoother() = default;

  private:
  };
} // namespace Planning
#endif // ! LOCAL_PATH_SMOOTHER_H_
