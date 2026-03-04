#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

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

namespace Planning
{
  static constexpr uint8 PROCESS_FAILURE{ 255U };

  using namespace ::std::chrono_literals; // for 1s block

  class PlanningProcess : public rclcpp::Node
  {
  public:
    PlanningProcess();
    ~PlanningProcess() = default;
    boolean process();

    inline base_msgs::msg::PNCMap getPNCMap() const { return pncMap; }

    inline nav_msgs::msg::Path getGlobalPath() const { return globalPath; }

  private:
    std::unique_ptr<ConfigReader> configReaderProcess;
    std::shared_ptr<VehicleInfoBase> egoCar;                   // ego car
    std::vector<std::shared_ptr<VehicleInfoBase>> TpCars;      // other traffic participants (cars)
    std::vector<std::shared_ptr<VehicleInfoBase>> TpCarsInROI; // other traffic participants (cars) in ROI
    float64 obsDis;                                            // obstacle distance

    // tf broadcaster: broadcast vehicle's initial information (e.g., pose info) to control module
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfBroadcaster;
    // tf listener: listen to the transform between map frame and vehicle frame from control module
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    base_msgs::msg::PNCMap pncMap;
    nav_msgs::msg::Path globalPath;
    // map client
    rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr pncMapClient;
    // global path client
    rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr globalPathClient;

    std::shared_ptr<ReferenceLineCreator> referenceLineCreator;                   // reference line creator
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr referenceLineRvizPublisher; // reference line rviz publisher
    rclcpp::TimerBase::SharedPtr runtime;                                         // runtime for planning process module

    boolean initPlanning();

    //// spawn vehicle, init and broadcast vehicle's pose
    void spawnVehicle(const std::shared_ptr<VehicleInfoBase>& vehicle);

    // listen to vehicle's real-time pose from control module
    void getVehicleLocation(const std::shared_ptr<VehicleInfoBase>& vehicle);

    // callback function for planning process
    void planningCallback();

    template <typename T>
    boolean connectServer(const T& client); // pnc map or global path client

    boolean requestPNCMap();
    boolean requestGlobalPath();
  };
} // namespace Planning
#endif // !PLANNING_PROCESS_H_