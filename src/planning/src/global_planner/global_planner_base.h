#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    enum class GlobalPlannerType//全局规划类型
    {
        NORMAL,
    };
    class GlobalPlannerBase //全局路径规划器基类
    {
    public:
        virtual Path search_global_path(const PNCMap &pnc_map) = 0; //搜索全局路径
        inline Path global_path() const{ return global_path_;}//获取全局路径
        virtual ~GlobalPlannerBase() {};
    
    protected:
        std::unique_ptr<ConfigReader> global_planner_config_; //配置
        int global_planner_type_ = 0; //类型
        Path global_path_;//全局路径

    };
}  // namespace Planning
#endif  // GLOBAL_PLANNER_BASE_H_
