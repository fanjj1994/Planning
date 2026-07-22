#include "planning_process.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
  {
    RCLCPP_INFO(this->get_logger(), "planning_process created.");

    // 读取配置文件
    process_config_ = std::make_unique<ConfigReader>();
    process_config_->read_planning_process_config();
    obs_dis_ = process_config_->process().obs_dis_;

    // 创建车辆和障碍物
    car_ = std::make_shared<MainCar>();
    for(int i = 0; i < 3 ; i++)
    {
      auto obs_car_ = std::make_shared<ObsCar>(i+1);
      obses_spawn_.emplace_back(obs_car_);
    }

    // 坐标广播器
    tf_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);

    // 创建监听器，绑定主车缓存对象
    buffer_ = std::make_unique<Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

    // 创建地图和全局路径客户端
    map_client_ = this->create_client<PNCMapService>("pnc_map_server");
    global_path_client_ = this->create_client<GlobalPathService>("global_path_server");

    // 创建参考线和参考线的发布器
    refer_line_creator_ = std::make_shared<ReferencelineCreator>();
    refer_line_pub_ = this->create_publisher<Path>("reference_line", 10);
  }

  bool PlanningProcess::process() // 总流程
  {
    // 阻塞1s，等待rviz和xacro模型先启动
    rclcpp::Rate rate(0.5);
    rate.sleep();

    // 初始化
    if (!planning_init())
    {
      RCLCPP_ERROR(this->get_logger(), "planning init failed");
      return false;
    }
    // 进入规划主流程
    timer_ = this->create_wall_timer(0.1s, std::bind(&PlanningProcess::planning_callback, this));

    return true;
  }

  bool PlanningProcess::planning_init() // 流程初始化
  {
    // 生成车辆
    vehicle_spawn(car_);
    for(const auto &obs : obses_spawn_)
    {
      vehicle_spawn(obs);
    }

    // 连接地图服务器
    if (!connect_server(map_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Map Server connect failed!");
      return false;
    }

    // 获取地图
    if (!map_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Map request and response failed!");
      return false;
    }

    // 连接全局路径服务器
    if (!connect_server(global_path_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Global_path server connect failed!");
      return false;
    }

    // 获取全局路径
    if (!global_path_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Global_path request and response failed!");
      return false;
    }

    return true;
  }

  void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle) // 生成车辆
  {
    TransformStamped spawn;
    spawn.header.stamp = this->now();
    spawn.header.frame_id =  process_config_->pnc_map().frame_; // 地图坐标
    spawn.child_frame_id = vehicle->child_frame(); // 地图坐标的子坐标设为车辆坐标

    spawn.transform.translation.x = vehicle->loc_point().pose.position.x;
    spawn.transform.translation.y = vehicle->loc_point().pose.position.y;
    spawn.transform.translation.z = vehicle->loc_point().pose.position.z;
    spawn.transform.rotation.x = vehicle->loc_point().pose.orientation.x;
    spawn.transform.rotation.y = vehicle->loc_point().pose.orientation.y;
    spawn.transform.rotation.z = vehicle->loc_point().pose.orientation.z;
    spawn.transform.rotation.w = vehicle->loc_point().pose.orientation.w;

    RCLCPP_INFO(this->get_logger(),"vehile %s spawned, x = %.2f, y = %.2f",
                spawn.child_frame_id.c_str(),
                vehicle->loc_point().pose.position.x,
                vehicle->loc_point().pose.position.y);
    tf_broadcaster_->sendTransform(spawn);
  }

  void PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle) // 监听定位点
  {
    try
    {
      PoseStamped point;
      auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_,vehicle->child_frame(),tf2::TimePointZero);
      point.header = ts.header;
      point.pose.position.x = ts.transform.translation.x;
      point.pose.position.y = ts.transform.translation.y;
      point.pose.position.z = ts.transform.translation.z;
      point.pose.orientation.x = ts.transform.rotation.x;
      point.pose.orientation.y = ts.transform.rotation.y;
      point.pose.orientation.z = ts.transform.rotation.z;
      point.pose.orientation.w = ts.transform.rotation.w;
      vehicle->update_location(point);
    }
    catch(const tf2::LookupException& e)
    {
      RCLCPP_ERROR(this->get_logger(),"Lookup exception: %s",e.what());
    }
    
  }

  template <typename T>
  bool PlanningProcess::connect_server(const T &client) // 连接服务器
  {
    // 判断客户端类型
    std::string server_name;
    if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)
    {
      server_name = "pnc_map";
    }
    else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
    {
      server_name = "global_path";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "wrong client type!");
      return false;
    }

    // 等待服务器
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok()) // 对ctrl+c操作处理。防止进入死循环
      {
        RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", server_name.c_str());
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "%s Server not available, waiting again...", server_name.c_str());
    }

    return true;
  }

  bool PlanningProcess::map_request() // 发送地图请求
  {
    RCLCPP_INFO(this->get_logger(), "Sending map request/");

    // 生成请求
    auto request = std::make_shared<PNCMapService::Request>();
    request->map_type = process_config_->pnc_map().type_;

    // 获取响应
    auto result_future = map_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Map response success");
      pnc_map_ = result_future.get()->pnc_map; // 获取响应中的pnc_map
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Map response failed!");
      return false;
    }

    return false;
  }

  bool PlanningProcess::global_path_request() // 发送全局路径请求
  {
    RCLCPP_INFO(this->get_logger(), "Sending Global path request/");
    // 生成请求
    auto request = std::make_shared<GlobalPathService::Request>();
    request->pnc_map = pnc_map_;
    request->global_planner_type = process_config_->global_path().type_;

    // 获取响应
    auto result_future = global_path_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Global path response success");
      global_path_ = result_future.get()->global_path; // 获取响应中的global_path
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Global path response failed!");
      return false;
    }

    return false;
  }
  void PlanningProcess::planning_callback() // 总流程回调
  {
    const auto start_time = this->get_clock()->now();
    // 监听车辆定位
    get_location(car_);

    // 参考线
    const auto refer_line = refer_line_creator_->creat_reference_line(global_path_,car_->loc_point());
    if(refer_line.refer_line.empty())
    {
      RCLCPP_ERROR(this->get_logger(),"reference line empty");
      return;
    }
    const auto refer_line_rviz = refer_line_creator_->referline_to_rviz(); //生成rviz用的参考线
    refer_line_pub_->publish(refer_line_rviz);// 发布参考线

    // 主车和障碍物向参考线投影

    // 障碍物按s值排序

    // 路径决策

    // 路径规划

    // 障碍物向路径投影

    // 速度决策

    // 速度规划

    // 合成轨迹

    // 更新绘图信息

    // 更新车辆信息

    RCLCPP_INFO(this->get_logger(), "-----------car state : loc:(%.2f,%.2f),speed:%.2f,a:%.2f,theta:%.2f,kappa:%.2f",
                car_->loc_point().pose.position.x, car_->loc_point().pose.position.y, car_->speed(),
                car_->acceleration(), car_->theta(), car_->kappa());

    const auto end_time = this->get_clock()->now();
    const double planning_total_time = end_time.seconds() - start_time.seconds();
    RCLCPP_INFO(this->get_logger(),"planning total time : %fms\n",planning_total_time * 1000);
    // 防止系统卡死
    if(planning_total_time > 1.0)
    {
      RCLCPP_ERROR(this->get_logger(),"planning time too long");
      rclcpp::shutdown();
    }
  }
} // namespace Planning
