#include "config_reader.h"

namespace Planning {
  ConfigReader::ConfigReader()
  {
    // get the path of workspace/install/planning_core/share/planning_core
    std_string configFilePath =
        ament_index_cpp::get_package_share_directory("planning_core");

    // get config file
    planningConfig = YAML::LoadFile(configFilePath + "/config/planning_static_obs_config.yaml");
  
  }

  void ConfigReader::readVehicleConfig(VehicleStruct &vehicle, const std_string &vehicleName)
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
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load VehicleConfig: %s", e.what());
    }
  }

} // namespace Planning
