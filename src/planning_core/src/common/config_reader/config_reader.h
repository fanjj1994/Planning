#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "common_type.h"
#include <unordered_map>

namespace Planning {
  struct VehicleStruct
  {
    uint8           id_{ 0U };
    std_string      frame_{ "" };
    float32         length_{ 0.0F };    // uint: m
    float32         width_{ 0.0F };     // uint: m
    float32         pose_x_{ 0.0F };    // uint: m
    float32         pose_y_{ 0.0F };    // uint: m
    float32         pose_theta_{ 0.0F };
    float32         speed_init_{ 0.0F };
  };

  struct PNCMapStruct
  {
    std_string      frame_{ "" };
    uint8           type_{ 0U };
    float32         road_length{ 0.0F };    // uint: m
    float32         road_half_width{ 0.0F };  // uint: m
    float32         segment_len{ 0.0F };    // uint: m
    float32         speed_limit{ 0.0F };    // uint: m/s
  };

  struct ReferenceLineStruct
  {
    uint8           type_{ 0U };
    uint32          front_size_{ 0U };
    uint32          back_size_{ 0U };
  };

  struct GlobalPathStruct
  {
    uint8           type_{ 0U };
  };

  struct LocalPathStruct
  {
    uint8           curve_type_{ 0U };
    uint32          path_size_{ 0U };
  };

  struct LocalSpeedsStruct
  {
    uint32          speeds_size_{ 0U };
  };

  struct DecisionStruct
  {
    float32        safe_dis_lat_{ 0.0F };    // uint: m
    float32        safe_dis_lon_{ 0.0F };    // uint: m
  };

  struct ProcessStruct
  {
    float32       obs_dis_{ 0.0F };    // uint: s
  };

  class ConfigReader
  {
  public:
    ConfigReader();
    ~ConfigReader() = default;

  private:
    // vehicle
    static VehicleStruct           egoCar;
    static VehicleStruct           tpCar1;
    static VehicleStruct           tpCar2;
    static VehicleStruct           tpCar3;

    static std::unordered_map<uint8, VehicleStruct> VehiclePairs;

    // pnc_map
    static PNCMapStruct            pncMap;

    // reference_line
    static ReferenceLineStruct     referenceLine;

    // global_path
    static GlobalPathStruct        globalPath;

    // local_path
    static LocalPathStruct         localPath;

    // local_speeds
    static LocalSpeedsStruct       localSpeeds;

    // decision
    static DecisionStruct          decision;

    // process
    static ProcessStruct           process;
  };
} // namespace Planning
#endif // !CONFIG_READER_H_
