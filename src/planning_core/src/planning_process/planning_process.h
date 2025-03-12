#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "common_type.h"
#include "config_reader.h"
#include "ego_car_base.h"
#include "tp_base.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning {
  static constexpr uint8 PROCESS_FAILURE{ 255U };

  using namespace::std::chrono_literals;

  class PlanningProcess : public rclcpp::Node
  {
  public:
    PlanningProcess();
    ~PlanningProcess() = default;
    boolean process();

    inline base_msgs::msg::PNCMap getPNCMap() const
    {
      return pncMap;
    }

    inline nav_msgs::msg::Path getGlobalPath() const
    {
      return globalPath;
    }

  private:
    std::unique_ptr<ConfigReader> configReaderProcess;
    float64 obsDis; // obstacle distance
    base_msgs::msg::PNCMap pncMap;
    nav_msgs::msg::Path globalPath;
    // map client
    rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr pncMapClient;
    // global path client
    rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr globalPathClient;

    boolean initPlanning();
    
    template <typename T>
    boolean connectServer(const T &client); // pnc map or global path client

    boolean requestPNCMap();
    boolean requestGlobalPath();
  };
} // namespace Planning
#endif // !PLANNING_PROCESS_H_