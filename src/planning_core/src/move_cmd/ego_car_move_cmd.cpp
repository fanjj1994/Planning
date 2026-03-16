#include "ego_car_move_cmd.h"

namespace Planning
{
  EgoCarMoveCmd::EgoCarMoveCmd() : Node("ego_car_move_cmd_node"), carParam{}
  {
    RCLCPP_INFO(this->get_logger(), "EgoCarMoveCmd Node has been created.");

    // Read config file for ego car move command component
    configReaderCarMoveCmd = std::make_unique<ConfigReader>();
    configReaderCarMoveCmd->readMoveCmdConfig();

    // Initialize the ego car's state information
    egoCar = std::make_shared<EgoCar>();
    carParam.posX = egoCar->getVehiclePose().pose.position.x;
    carParam.posY = egoCar->getVehiclePose().pose.position.y;
    carParam.theta = egoCar->getVehicleTheta();
    carParam.speed = egoCar->getVehicleVelocity();

    // Initialize the transform broadcaster
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Initialize the local trajectory subscription
    localTrajectorySubscription = this->create_subscription<base_msgs::msg::LocalTrajectory>(
        "planning_core/local_trajectory", 10, std::bind(&EgoCarMoveCmd::carBroadcastTf, this, std::placeholders::_1));
  }

  void EgoCarMoveCmd::carBroadcastTf(const base_msgs::msg::LocalTrajectory::SharedPtr trajectoryMsg)
  {
    // receive the local trajectory
    const uint8 trajectorySize = static_cast<uint8>(trajectoryMsg->local_trajectory.size());

    if (trajectorySize < 3U)
    {
      RCLCPP_WARN(this->get_logger(), "Received local trajectory with insufficient points: %u", trajectorySize);
      return;
    }

    // create msg
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = trajectoryMsg->header.stamp;
    transformStamped.header.frame_id = configReaderCarMoveCmd->getPNCMap().frame_; // map frame as the parent frame
    transformStamped.child_frame_id = egoCar->getVehicleChildFrame();

    // compute projected point; for simplicity, use the closest match point
    float64 minDistance = std::numeric_limits<float64>::max();
    uint8 closestPointIndex = 0U;
    for (uint8 i = 0U; i < trajectorySize; i++)
    {
      const float64 dx = trajectoryMsg->local_trajectory.at(i).path_point.pose.pose.position.x - carParam.posX;
      const float64 dy = trajectoryMsg->local_trajectory.at(i).path_point.pose.pose.position.y - carParam.posY;
      const float64 distance = std::hypot(dx, dy);
      if (distance < minDistance)
      {
        minDistance = distance;
        closestPointIndex = i;
      }
    }

    // value given
    const float64 speed_x = trajectoryMsg->local_trajectory.at(closestPointIndex).speed_point.speed *
                            std::cos(trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.theta);
    const float64 speed_y = trajectoryMsg->local_trajectory.at(closestPointIndex).speed_point.speed *
                            std::sin(trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.theta);

#ifdef USE_ACTUAL_POS
    // use vehicle state information to update the transform of ego car
    transformStamped.transform.translation.x = carParam.posX;
    transformStamped.transform.translation.y = carParam.posY;
    transformStamped.transform.translation.z = 0.0; // assuming flat ground
#else
    // use match point to update the transform of ego car
    transformStamped.transform.translation.x =
        trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.pose.pose.position.x;
    transformStamped.transform.translation.y =
        trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.pose.pose.position.y;
#endif // USE_ACTUAL_POS

    // update vehicle state parameter
    carParam.posX += speed_x; // update posX based on the speed in x direction
    carParam.posY += speed_y; // update posY based on the speed in y direction

    // update the transform of ego car based on the updated state parameter
    transformStamped.transform.rotation.x =
        trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.pose.pose.orientation.x;
    transformStamped.transform.rotation.y =
        trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.pose.pose.orientation.y;
    transformStamped.transform.rotation.z =
        trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.pose.pose.orientation.z;
    transformStamped.transform.rotation.w =
        trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.pose.pose.orientation.w;

    // broadcast the transform of ego car based on the updated state parameter
    RCLCPP_INFO(this->get_logger(),
                "Broadcasting transform for ego car: posX = %.2f, posY = %.2f, theta = %.2f, speed = %.2f",
                carParam.posX, carParam.posY, trajectoryMsg->local_trajectory.at(closestPointIndex).path_point.theta,
                std::hypot(speed_x, speed_y));
    tfBroadcaster->sendTransform(transformStamped);
  }
} // namespace Planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::EgoCarMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
