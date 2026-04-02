#include "planning_process.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process")
  {
    // Initialize the planning process
    RCLCPP_INFO(this->get_logger(), "PlanningProcess created!");

    // Read config file for planning process
    configReaderProcess = std::make_unique<ConfigReader>();
    configReaderProcess->readProcessConfig();
    perceptionRange = configReaderProcess->getProcess().perception_range_;

    // create ego car and other tps
    egoCar = std::make_shared<EgoCar>();

    for (uint i = 0U; i < configReaderProcess->getScenario().tp_num_; i++)
    {
      // traffic participant car, ID starts from 1
      std::shared_ptr<VehicleInfoBase> tpCar = std::make_shared<TP>(i + 1U);
      TpCars.emplace_back(tpCar);
    }

    // broadcast initial pose for vehicles
    tfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // create tf listener to get vehicle's real-time pose from control module
    tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // create PNCMap & Global Path Client
    pncMapClient = this->create_client<base_msgs::srv::PNCMapService>("pnc_map_server");
    globalPathClient = this->create_client<base_msgs::srv::GlobalPathService>("global_path_server");

    // create reference line, reference line rviz publisher
    referenceLineCreator = std::make_shared<ReferenceLineCreator>();
    referenceLineRvizPublisher = this->create_publisher<nav_msgs::msg::Path>("reference_line", 10);

    // create decision center
    decisionCenter = std::make_shared<DecisionCenter>();

    // create local path planner
    localPathPlanner = std::make_shared<LocalPathPlanner>();

    // create local speeds planner
    localSpeedsPlanner = std::make_shared<LocalSpeedsPlanner>();

    // create local path publisher
    localPathPublisher = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);

    // create local trajectory combiner
    localTrajectoryCombiner = std::make_shared<LocalTrajectoryCombiner>();

    // create local trajectory publisher
    localTrajectoryPublisher = this->create_publisher<base_msgs::msg::LocalTrajectory>("local_trajectory", 10);

    // create data plot publisher
    dataPlotPublisher = this->create_publisher<base_msgs::msg::PlotInfo>("plot_info", 10);
  }

  boolean PlanningProcess::process()
  {
    // set 1 second timer to wait rviz2 & xacro to be ready
    rclcpp::Rate rate(1.0);
    rate.sleep();

    if (initPlanning() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "planning init failed!");
      return false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "init complete. Start planning process main function!");
    }

    // planning main process
    runtime = this->create_wall_timer(100ms, std::bind(&PlanningProcess::planningCallback, this)); // runtime 100ms
    return true;
  }

  boolean PlanningProcess::initPlanning()
  {
    boolean initPlanningResult = true;

    // create tps
    for (const auto& tpCar : TpCars)
    {
      spawnVehicle(tpCar);
    }

    // create ego car
    spawnVehicle(egoCar);

    // connect map server
    if (connectServer(pncMapClient) == false)
    {
      RCLCPP_ERROR(this->get_logger(), "connect pnc map server failed!");
      initPlanningResult = false;
    }

    // get map
    if (requestPNCMap() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "request pnc map failed!");
      initPlanningResult = false;
    }

    // connect global path server
    if (connectServer(globalPathClient) == false)
    {
      RCLCPP_ERROR(this->get_logger(), "connect global path server failed!");
      initPlanningResult = false;
    }

    // get global path
    if (requestGlobalPath() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "request global path failed!");
      initPlanningResult = false;
    }

    return initPlanningResult;
  }

  void PlanningProcess::spawnVehicle(const std::shared_ptr<VehicleInfoBase>& vehicle)
  {
    geometry_msgs::msg::TransformStamped spawn;
    spawn.header.stamp = this->get_clock()->now();
    // map as the parent frame, vehicle frame as the child frame
    spawn.header.frame_id = configReaderProcess->getPNCMap().frame_;
    spawn.child_frame_id = vehicle->getVehicleChildFrame();

    spawn.transform.translation.x = vehicle->getVehiclePose().pose.position.x;
    spawn.transform.translation.y = vehicle->getVehiclePose().pose.position.y;
    spawn.transform.translation.z = vehicle->getVehiclePose().pose.position.z;
    spawn.transform.rotation.x = vehicle->getVehiclePose().pose.orientation.x;
    spawn.transform.rotation.y = vehicle->getVehiclePose().pose.orientation.y;
    spawn.transform.rotation.z = vehicle->getVehiclePose().pose.orientation.z;
    spawn.transform.rotation.w = vehicle->getVehiclePose().pose.orientation.w;
    RCLCPP_INFO(this->get_logger(), "vehicle %s is spawned, x = %.2f, y=%.2f", spawn.child_frame_id.c_str(),
                spawn.transform.translation.x, spawn.transform.translation.y);
    // broadcast the transform
    this->tfBroadcaster->sendTransform(spawn);
  }

  void PlanningProcess::getVehicleLocation(const std::shared_ptr<VehicleInfoBase>& vehicle)
  {
    try
    {
      geometry_msgs::msg::PoseStamped point; // temporary variable to store the vehicle's pose
      auto ts = tfBuffer->lookupTransform(configReaderProcess->getPNCMap().frame_, vehicle->getVehicleChildFrame(),
                                          tf2::TimePointZero); // get the latest transform
      point.header.stamp = ts.header.stamp;
      point.header.frame_id = ts.header.frame_id;
      point.pose.position.x = ts.transform.translation.x;
      point.pose.position.y = ts.transform.translation.y;
      point.pose.position.z = ts.transform.translation.z;
      point.pose.orientation.x = ts.transform.rotation.x;
      point.pose.orientation.y = ts.transform.rotation.y;
      point.pose.orientation.z = ts.transform.rotation.z;
      point.pose.orientation.w = ts.transform.rotation.w;
      vehicle->updateVehiclePose(point); // update vehicle's pose
    }
    catch (const tf2::LookupException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", e.what());
    }
  }

  // callback function for planning process
  void PlanningProcess::planningCallback()
  {
    const auto planningStartTime = this->get_clock()->now();
    // get vehicle's real-time pose from control module
    getVehicleLocation(egoCar);
    TpCarsInROI.clear();
    for (const auto& tpCar : TpCars)
    {
      getVehicleLocation(tpCar);
      if (std::hypot(egoCar->getVehiclePose().pose.position.x - tpCar->getVehiclePose().pose.position.x,
                     egoCar->getVehiclePose().pose.position.y - tpCar->getVehiclePose().pose.position.y) <=
          perceptionRange)
      {
        TpCarsInROI.emplace_back(tpCar);
      }
    }

    // create reference line
    const auto referenceLine_ = referenceLineCreator->createReferenceLine(globalPath, egoCar->getVehiclePose());
    if (referenceLine_.refer_line.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "reference line is empty!");
      return;
    }
    const auto referencelineRviz = referenceLineCreator->referenceLineToRviz();
    referenceLineRvizPublisher->publish(referencelineRviz); // publish

    // ego car, tps projected to the reference line
    egoCar->vehicleCartesianToFrenet(referenceLine_);
    for (const auto& tpCar : TpCarsInROI)
    {
      tpCar->vehicleCartesianToFrenet(referenceLine_);
    }

    // tps sort by s value
    std::sort(TpCarsInROI.begin(), TpCarsInROI.end(),
              [](const std::shared_ptr<VehicleInfoBase>& a, const std::shared_ptr<VehicleInfoBase>& b) {
                return a->getS() < b->getS();
              });

    // path decision making
    decisionCenter->makePathDecision(egoCar, TpCarsInROI);

    // local path planning
    // generate local path in Frenet coordinates
    const auto localPath_ = localPathPlanner->generateLocalPath(referenceLine_, decisionCenter, egoCar);
    if (localPath_.local_path.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "local path is empty!");
      return;
    }
    const auto localPathRviz = localPathPlanner->localPathToRviz();
    localPathPublisher->publish(localPathRviz); // publish

    // tps projected to the local path
    for (const auto& tpCar : TpCarsInROI)
    {
      tpCar->vehicleCartesianToFrenet2Path(localPath_, referenceLine_, egoCar);
    }

    // speeds decision making
    decisionCenter->makeSpeedDecision(egoCar, TpCarsInROI);

    // local speeds planning
    const auto localSpeeds_ = localSpeedsPlanner->planLocalSpeeds(decisionCenter);
    if (localSpeeds_.local_speeds.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "local speeds is empty!");
      return;
    }

    // compose trajectory
    const auto localTrajectory_ = localTrajectoryCombiner->combineLocalTrajectory(localPath_, localSpeeds_);
    if (localTrajectory_.local_trajectory.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "local trajectory is empty!");
      return;
    }
    localTrajectoryPublisher->publish(localTrajectory_); // publish local trajectory

    // update data plotting
    base_msgs::msg::PlotInfo plotInfo;
    plotInfo.header.stamp = this->get_clock()->now();
    plotInfo.header.frame_id = configReaderProcess->getPNCMap().frame_;
    plotInfo.trajectory_info = localTrajectory_;

    base_msgs::msg::ObsInfo tpInfo;
    for (const auto& tpCar : TpCarsInROI)
    {
      tpInfo.obs_length = tpCar->getVehicleLength();
      tpInfo.obs_width = tpCar->getVehicleWidth();
      tpInfo.l = tpCar->getL();
      tpInfo.s = tpCar->getS();
      tpInfo.s_2path = tpCar->getS2Path();
      tpInfo.ds_dt_2path = tpCar->getDsDt2Path();
      tpInfo.t_in = tpCar->getTIn();
      tpInfo.t_out = tpCar->getTOut();
      plotInfo.obs_info.emplace_back(tpInfo);
    }
    dataPlotPublisher->publish(plotInfo); // publish data plot

    // update vehicle's info
    egoCar->updateCartesianInfo(localTrajectory_.local_trajectory.front());
    RCLCPP_INFO(this->get_logger(), "----------------car state: loc: (%.2f, %.2f), speed: %.2f, a: %.2f, kappa: %.2f",
                egoCar->getVehiclePose().pose.position.x, egoCar->getVehiclePose().pose.position.y,
                egoCar->getVehicleVelocity(), egoCar->getVehicleAcceleration(), egoCar->getVehicleKappa());
    const auto planningEndTime = this->get_clock()->now();
    const float64 planningDuration =
        static_cast<float64>((planningEndTime.seconds() - planningStartTime.seconds()) * 1000.0); // ms
    RCLCPP_INFO(this->get_logger(), "Planning process duration: %.2f ms", planningDuration);

    // exception handling
    if (planningDuration > 1000.0)
    {
      RCLCPP_WARN(this->get_logger(), "Planning process duration exceeds 1 second!");
      rclcpp::shutdown();
    }
  }

  template <typename T>
  boolean PlanningProcess::connectServer(const T& client)
  {
    // classify client type
    std_string serverName;
    if constexpr (std::is_same_v<T, rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr> == true)
    {
      serverName = "pnc_map";
    }
    else if constexpr (std::is_same_v<T, rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr> == true)
    {
      serverName = "global_path";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "wrong client type!");
    }

    // wait_for_server
    while (!client->wait_for_service(1s)) // wait for 1 second
    {
      if (!rclcpp::ok()) // Ctrl+C to stop
      {
        RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", serverName.c_str());
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Server %s not available, waiting again...", serverName.c_str());
    }
    return true;
  }

  boolean PlanningProcess::requestPNCMap()
  {
    RCLCPP_INFO(this->get_logger(), "Send request to PNCMapServer");

    // send request
    auto request = std::make_shared<base_msgs::srv::PNCMapService::Request>();
    request->map_type = configReaderProcess->getPNCMap().type_;

    // receive response
    auto result = pncMapClient->async_send_request(request);

    // identify response status
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Map server responds successfully!");
      pncMap = result.get()->pnc_map; // get the map from the response
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive response from PNCMapServer");
      return false;
    }
    return false;
  }

  boolean PlanningProcess::requestGlobalPath()
  {
    RCLCPP_INFO(this->get_logger(), "Send request to GlobalPathServer");

    // send request
    auto request = std::make_shared<base_msgs::srv::GlobalPathService::Request>();
    request->pnc_map = pncMap;
    request->global_planner_type = configReaderProcess->getGlobalPath().type_;

    if (request->pnc_map.midline.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "PNCMap is empty before sending to GlobalPathServer");
    }

    // receive response
    auto result = globalPathClient->async_send_request(request);

    // identify response status
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Global path server responds successfully!");
      globalPath = result.get()->global_path; // get the global path from the response
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive response from GlobalPathServer");
      return false;
    }
    return false;
  }

} // namespace Planning
