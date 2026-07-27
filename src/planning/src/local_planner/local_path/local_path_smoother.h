#ifndef LOCAL_PATH_SMOOTHER_H_
#define LOCAL_PATH_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include <cmath>
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::LocalPath;
    
    class LocalPathSmoother //局部路径平滑器
    {
    public:
        LocalPathSmoother(); 
        void smooth_local_path(LocalPath &path); //平滑路径
    private:
        std::unique_ptr<ConfigReader> local_path_config_; // 配置
    };
}  // namespace Planning
#endif  // LOCAL_PATH_SMOOTHER_H_   
