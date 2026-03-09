#ifndef LOCAL_PATH_SMOOTHER_H_
#define LOCAL_PATH_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/local_path.hpp"
#include <cmath>

namespace Planning
{
  class LocalPathSmoother
  {
  public:
    LocalPathSmoother();
    LocalPathSmoother(const LocalPathSmoother &) = delete;
    LocalPathSmoother operator=(const LocalPathSmoother &) = delete;
    ~LocalPathSmoother() = default;

    void smoothLocalPath(base_msgs::msg::LocalPath &localPath);

  private:
    std::unique_ptr<ConfigReader> localPathConfigReader; // config reader for local path smoother
  };
} // namespace Planning
#endif // ! LOCAL_PATH_SMOOTHER_H_
