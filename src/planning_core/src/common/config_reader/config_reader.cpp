#include "config_reader.h"

namespace Planning
{
  ConfigReader::ConfigReader()
  {
    // get the path of workspace/install/planning_core/share/planning_core
    std_string configFilePath = ament_index_cpp::get_package_share_directory("planning_core");

    // get config file
    scenario_config = YAML::LoadFile(configFilePath + "/config/scenario_config.yaml");

    // figure out scenario type
    readScenarioConfig();

    switch (scenario.type_)
    {
    case ScenarioType::FOLLOW_LANE:
    case ScenarioType::STATIC_TP_DETOUR:
      planningConfig = YAML::LoadFile(configFilePath + "/config/planning_static_tps_config.yaml");
      break;
    case ScenarioType::TP_IN_EGO_LANE:
      planningConfig = YAML::LoadFile(configFilePath + "/config/planning_inlane_tps_config.yaml");
      break;
    case ScenarioType::DYNAMIC_TP_DETOUR:
      planningConfig = YAML::LoadFile(configFilePath + "/config/planning_dynamic_tps_config.yaml");
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Invalid scenario type: %d", static_cast<uint8>(scenario.type_));
      break;
    }
  }

  void ConfigReader::readScenarioConfig()
  {
    try
    {
      scenario.type_ = static_cast<ScenarioType>(scenario_config["scenario"]["type"].as<uint8>());
      scenario.tp_num_ = scenario_config["scenario"]["tp_num"].as<uint8>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load ScenarioConfig: %s", e.what());
    }
  }

  void ConfigReader::readVehicleConfig(VehicleStruct& vehicle, const std_string& vehicleName)
  {
    vehicle.id_ = planningConfig["vehicle"][vehicleName]["id"].as<uint8>();
    vehicle.frame_ = planningConfig["vehicle"][vehicleName]["frame"].as<std_string>();
    vehicle.length_ = planningConfig["vehicle"][vehicleName]["length"].as<float32>();
    vehicle.width_ = planningConfig["vehicle"][vehicleName]["width"].as<float32>();
    vehicle.pose_x_ = planningConfig["vehicle"][vehicleName]["pose_x"].as<float32>();
    vehicle.pose_y_ = planningConfig["vehicle"][vehicleName]["pose_y"].as<float32>();
    vehicle.pose_theta_ = planningConfig["vehicle"][vehicleName]["pose_theta"].as<float32>();
    vehicle.set_speed_ = planningConfig["vehicle"][vehicleName]["set_speed"].as<float32>();

    if (vehicleName != "ego_car")
    {
      VehiclePairs.emplace(vehicle.id_, vehicle);
    }
  }

  void ConfigReader::readVehiclesConfig()
  {
    try
    {
      readPNCMapConfig();
      readVehicleConfig(egoCar, "ego_car");
      readVehicleConfig(tpCar1, "tp_car1");
      readVehicleConfig(tpCar2, "tp_car2");
      readVehicleConfig(tpCar3, "tp_car3");
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load VehicleConfig: %s", e.what());
    }
  }

  void ConfigReader::readPNCMapConfig()
  {
    try
    {
      pncMap.frame_ = planningConfig["pnc_map"]["frame"].as<std_string>();
      pncMap.type_ = planningConfig["pnc_map"]["type"].as<uint8>();
      pncMap.road_length_ = planningConfig["pnc_map"]["road_length"].as<float32>();
      pncMap.lane_width_ = planningConfig["pnc_map"]["lane_width"].as<float32>();
      pncMap.segment_len_ = planningConfig["pnc_map"]["segment_len"].as<float32>();
      pncMap.speed_limit_ = planningConfig["pnc_map"]["speed_limit"].as<float32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load PNCMapConfig: %s", e.what());
    }
  }

  void ConfigReader::readGlobalPathConfig()
  {
    try
    {
      globalPath.type_ = planningConfig["global_path"]["type"].as<uint8>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load GlobalPathConfig: %s", e.what());
    }
  }

  void ConfigReader::readReferenceLineConfig()
  {
    try
    {
      readPNCMapConfig();
      referenceLine.type_ = planningConfig["reference_line"]["type"].as<uint8>();
      referenceLine.front_size_ = planningConfig["reference_line"]["front_size"].as<uint32>();
      referenceLine.back_size_ = planningConfig["reference_line"]["back_size"].as<uint32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load ReferenceLineConfig: %s", e.what());
    }
  }

  void ConfigReader::readLocalPathConfig()
  {
    try
    {
      readPNCMapConfig();
      readReferenceLineConfig();
      localPath.curve_type_ = planningConfig["local_path"]["curve_type"].as<uint8>();
      localPath.path_size_ = planningConfig["local_path"]["path_size"].as<uint32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load LocalPathConfig: %s", e.what());
    }
  }

  void ConfigReader::readLocalSpeedsConfig()
  {
    try
    {
      readVehiclesConfig();
      localSpeeds.speeds_size_ = planningConfig["local_speeds"]["speeds_size"].as<uint32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load LocalSpeedsConfig: %s", e.what());
    }
  }

  void ConfigReader::readDecisionConfig()
  {
    try
    {
      readPNCMapConfig();
      readReferenceLineConfig();
      readLocalPathConfig();
      readLocalSpeedsConfig();

      decision.lat_safe_margin_ = planningConfig["decision"]["lat_safe_margin"].as<float32>();
      decision.long_safe_margin_ = planningConfig["decision"]["long_safe_margin"].as<float32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load DecisionConfig: %s", e.what());
    }
  }

  void ConfigReader::readProcessConfig()
  {
    try
    {
      readPNCMapConfig();
      readGlobalPathConfig();
      readVehiclesConfig();

      process.perception_range_ = planningConfig["planning_process"]["perception_range"].as<float32>();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load ProcessConfig: %s", e.what());
    }
  }

  void ConfigReader::readMoveCmdConfig()
  {
    try
    {
      readPNCMapConfig();
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load MoveCmdConfig: %s", e.what());
    }
  }

} // namespace Planning
