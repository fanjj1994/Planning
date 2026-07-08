#include "config_reader.h"
#include <fstream>

namespace Planning
{
  ConfigReader::ConfigReader() // 配置文件读取器
  {
    // 获取/workspace/install/planning/share/planning/目录路径
    std::string planning_share_directory = ament_index_cpp ::get_package_share_directory("planning");

    // 然后获取配置文件
    planning_config = YAML::LoadFile(planning_share_directory + "/config/planning_static_obs_config.yaml");
  }

  /************ vehicle ************/
  void ConfigReader::read_vehicle_config(VehicleStruct &vehicle, const std::string &name)
  {
    vehicle.id_ = planning_config["vehicle"][name]["id"].as<int>();
    vehicle.frame_ = planning_config["vehicle"][name]["frame"].as<std::string>();
    vehicle.length_ = planning_config["vehicle"][name]["length"].as<double>();
    vehicle.width_ = planning_config["vehicle"][name]["width"].as<double>();
    vehicle.pose_x_ = planning_config["vehicle"][name]["pose_x"].as<double>();
    vehicle.pose_y_ = planning_config["vehicle"][name]["pose_y"].as<double>();
    vehicle.pose_theta_ = planning_config["vehicle"][name]["pose_theta"].as<double>();
    vehicle.speed_ori_ = planning_config["vehicle"][name]["speed_ori"].as<double>();

    if (name != "main_car")
    {
      obs_pair_.emplace(vehicle.id_, vehicle);
    }
  }

  void ConfigReader::read_vehicles_config()
  {
    try
    {
      read_pnc_map_config();
      read_vehicle_config(main_car_, "main_car");
      read_vehicle_config(obs_car1_, "obs_car1");
      read_vehicle_config(obs_car2_, "obs_car2");
      read_vehicle_config(obs_car3_, "obs_car3");
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load vehicle config: %s", e.what());
    }
  }

  /*********** pnc_map *****************/
  void ConfigReader::read_pnc_map_config()
  {
    try
    {
      pnc_map_.frame_ = planning_config["pnc_map"]["frame"].as<std::string>();
      pnc_map_.type_ = planning_config["pnc_map"]["type"].as<int>();
      pnc_map_.road_length_ = planning_config["pnc_map"]["road_length"].as<double>();
      pnc_map_.road_half_width_ = planning_config["pnc_map"]["road_half_width"].as<double>();
      pnc_map_.segment_len_ = planning_config["pnc_map"]["segment_len"].as<double>();
      pnc_map_.speed_limit_ = planning_config["pnc_map"]["speed_limit"].as<double>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load pnc_map config");
    }
  }

  /************ global_path **********************/
  void ConfigReader::read_global_path_config()
  {
    try
    {
      global_path_.type_ = planning_config["global_path"]["type"].as<int>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load global_path config");
    }
  }

  /*************** reference_line ******************/
  void ConfigReader::read_reference_line_config()
  {
    try
    {
      read_pnc_map_config();
      refer_line_.type_ = planning_config["reference_line"]["type"].as<int>();
      refer_line_.front_size_ = planning_config["reference_line"]["front_size"].as<int>();
      refer_line_.back_size_ = planning_config["reference_line"]["back_size"].as<int>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load reference_line config");
    }
  }

  /**************** local_path **************/
  void ConfigReader::read_local_path_config()
  {
    try
    {
      read_pnc_map_config();
      read_reference_line_config();
      local_path_.curve_type_ = planning_config["local_path"]["curve_type"].as<int>();
      local_path_.path_size_ = planning_config["local_path"]["path_size"].as<int>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load local_path config");
    }
  }

  /*************** local_speeds ********************/
  void ConfigReader::read_local_speeds_config()
  {
    try
    {
      read_pnc_map_config();
      local_speeds_.speed_size_ = planning_config["local_speed"]["speed_size"].as<int>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load local_speeds config");
    }
  }

  /************ decision ******************/
  void ConfigReader::read_decision_config()
  {
    try
    {
      read_pnc_map_config();
      read_reference_line_config();
      read_local_path_config();
      read_local_speeds_config();
      decision_.safe_dis_l_ = planning_config["decision"]["safe_dis_l"].as<double>();
      decision_.safe_dis_s_ = planning_config["decision"]["safe_dis_s"].as<double>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("decision"), "Failed to load decision config");
    }
  }

  /******************* planning_process ******************/
  void ConfigReader::read_planning_process_config()
  {
    try
    {
      read_pnc_map_config();
      read_global_path_config();
      read_vehicles_config();
      process_.obs_dis_ = planning_config["planning_process"]["obs_dis"].as<double>();
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load planning_process config:%s",e.what());
    }
  }

  /************** move_cmd ***************/
  void ConfigReader:: read_move_cmd_config()
  {
    try
    {
        read_pnc_map_config();
    }
    catch(const YAML::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("config"),"Failed to load move_cmd config");
    }    
  }
} // namespace Planning