#include "tp_move_cmd.h"

namespace Planning
{
  TPMoveCmd::TPMoveCmd() : Node("tp_move_cmd_node")
  {
    RCLCPP_INFO(this->get_logger(), "TPMoveCmd Node has been created.");

    // read the parameters of tp cars for move command from config file
    configReaderTp = std::make_unique<ConfigReader>();
    configReaderTp->readMoveCmdConfig();

    // initialize tpCarParam
    for (uint8 i = 0U; i < 3U; i++)
    {
      TpParam tpCarParam;
      // traffic participant car, ID starts from 1
      tpCarParam.tpCar = std::make_shared<TP>(i + 1U);
      tpCarParam.tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      tpCarParam.posX = tpCarParam.tpCar->getVehiclePose().pose.position.x;
      tpCarParam.posY = tpCarParam.tpCar->getVehiclePose().pose.position.y;
      tpCarParam.theta = tpCarParam.tpCar->getVehicleTheta();
      tpCarParam.speed = tpCarParam.tpCar->getVehicleVelocity();
      tpCarParams.emplace_back(tpCarParam);
    }

    // move cmd for tp cars, control the frequency of tp car move command by timer
    timerTpMoveCmd =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TPMoveCmd::tpBroadcastTf, this));
  }

  void TPMoveCmd::tpBroadcastTf()
  {
    for (auto &tpCarParam : tpCarParams)
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->get_clock()->now();
      transformStamped.header.frame_id = configReaderTp->getPNCMap().frame_; // map frame as the parent frame
      transformStamped.child_frame_id = tpCarParam.tpCar->getVehicleChildFrame();

      // update the tp car's state information
      transformStamped.transform.translation.x = tpCarParam.posX;
      transformStamped.transform.translation.y = tpCarParam.posY;
      transformStamped.transform.translation.z = 0.0;

      const float64 speed_x = tpCarParam.speed * std::cos(tpCarParam.theta);
      const float64 speed_y = tpCarParam.speed * std::sin(tpCarParam.theta);
      // update the tp car's position based on its speed and heading direction
      tpCarParam.posX += speed_x;
      tpCarParam.posY += speed_y;

      tf2::Quaternion qtn;
      qtn.setRPY(0.0, 0.0, tpCarParam.theta);
      transformStamped.transform.rotation.x = qtn.getX();
      transformStamped.transform.rotation.y = qtn.getY();
      transformStamped.transform.rotation.z = qtn.getZ();
      transformStamped.transform.rotation.w = qtn.getW();
      // broadcast the transform of tp car
      RCLCPP_INFO(this->get_logger(),
                  "Broadcasting transform for TP car %s at position (%.2f, %.2f) with speed %.2f m/s",
                  tpCarParam.tpCar->getVehicleChildFrame().c_str(), tpCarParam.posX, tpCarParam.posY, tpCarParam.speed);
      tpCarParam.tfBroadcaster->sendTransform(transformStamped);
    }
  }
} // namespace Planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::TPMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
