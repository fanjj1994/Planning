#ifndef GLOABL_PLANNER_BASE_H_
#define GLOABL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "common_type.h"

namespace Planning
{

  enum class GlobalPlannerType : uint8
  {
    NORMAL = 0U,
    ASTAR = 1U,
    // ADD NEW PLANNER HERE
    DEFAULT = 255U
  };
  class GlobalPlannerBase
  {
  public:
    GlobalPlannerBase() = default;
    GlobalPlannerBase(const GlobalPlannerBase &) = delete;
    virtual ~GlobalPlannerBase() = default;

    virtual nav_msgs::msg::Path searchGlobalPath(const base_msgs::msg::PNCMap &) = 0;

    inline nav_msgs::msg::Path getGlobalPath() const { return globalPath; }

  protected:
    std::unique_ptr<ConfigReader> globalPathPlannerConfig;
    uint8 globalPlannerType{ 0U };
    nav_msgs::msg::Path globalPath;

  private:
  };
} // namespace Planning
#endif // ! GLOABL_PLANNER_BASE_H_
