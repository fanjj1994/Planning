#include "config_reader.h"

namespace Planning {
  ConfigReader::ConfigReader()
  {
    // get the path of workspace/install/planning_core/share/planning_core
    std_string configFilePath = ament_index_cpp::get_package_share_directory("planning_core");

    // get config file
    planningConfig = YAML::LoadFile(configFilePath + "/config/planning_static_obs_config.yaml");
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
    vehicle.speed_init_ = planningConfig["vehicle"][vehicleName]["speed_init"].as<float32>();

    if (vehicleName != "ego_car")
    {
      VehiclePairs.emplace(vehicle.id_, vehicle);
    }
  }

  void ConfigReader::readVehicleConfig()
  {
    try
    {
      readPNCMapConfig();
      readVehicleConfig(egoCar, "ego_car");
      readVehicleConfig(tpCar1, "obs_car1");
      readVehicleConfig(tpCar2, "obs_car2");
      readVehicleConfig(tpCar3, "obs_car3");
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
      pncMap.road_half_width_ = planningConfig["pnc_map"]["road_half_width"].as<float32>();
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
      readPNCMapConfig();
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

      decision.safe_dis_lat_ = planningConfig["decision"]["safe_dis_lat"].as<float32>();
      decision.safe_dis_lon_ = planningConfig["decision"]["safe_dis_lon"].as<float32>();
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
      readVehicleConfig();

      process.obs_dis_ = planningConfig["planning_process"]["obs_dis"].as<float32>();
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
