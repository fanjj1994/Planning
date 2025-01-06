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
    uint8 id_{ 0U };
    std_string frame_{ "" };
    float32 length_{ 0.0F }; // uint: m
    float32 width_{ 0.0F };  // uint: m
    float32 pose_x_{ 0.0F }; // uint: m
    float32 pose_y_{ 0.0F }; // uint: m
    float32 pose_theta_{ 0.0F };
    float32 speed_init_{ 0.0F };
  };

  struct PNCMapStruct
  {
    std_string frame_{ "" };
    uint8 type_{ 0U };
    float32 road_length{ 0.0F };     // uint: m
    float32 road_half_width{ 0.0F }; // uint: m
    float32 segment_len{ 0.0F };     // uint: m
    float32 speed_limit{ 0.0F };     // uint: m/s
  };

  struct ReferenceLineStruct
  {
    uint8 type_{ 0U };
    uint32 front_size_{ 0U };
    uint32 back_size_{ 0U };
  };

  struct GlobalPathStruct
  {
    uint8 type_{ 0U };
  };

  struct LocalPathStruct
  {
    uint8 curve_type_{ 0U };
    uint32 path_size_{ 0U };
  };

  struct LocalSpeedsStruct
  {
    uint32 speeds_size_{ 0U };
  };

  struct DecisionStruct
  {
    float32 safe_dis_lat_{ 0.0F }; // uint: m
    float32 safe_dis_lon_{ 0.0F }; // uint: m
  };

  struct ProcessStruct
  {
    float32 obs_dis_{ 0.0F }; // uint: s
  };

  class ConfigReader
  {
  public:
    ConfigReader();
    ConfigReader(const ConfigReader &) = delete;
    ConfigReader &operator=(const ConfigReader &) = delete;
    ~ConfigReader() = default;

    // vehicle
    void readVehicleConfig();

    void readVehicleConfig(VehicleStruct &vehicle, const std_string &vehicleName);

    inline VehicleStruct getEgoCar() const { return egoCar; }

    inline VehicleStruct getTpCar1() const { return tpCar1; }

    inline VehicleStruct getTpCar2() const { return tpCar2; }

    inline VehicleStruct getTpCar3() const { return tpCar3; }

    inline std::unordered_map<uint8, VehicleStruct> getVehiclePairs() const { return VehiclePairs; }

    // pnc_map
    void readPNCMapConfig();

    inline PNCMapStruct getPNCMap() const { return pncMap; }

    // global path planning
    void readGlobalPathConfig();

    inline GlobalPathStruct getGlobalPath() const { return globalPath; }

    // reference line
    void readReferenceLineConfig();

    inline ReferenceLineStruct getReferenceLine() const { return referenceLine; }

    // local path planning
    void readLocalPathConfig();

    inline LocalPathStruct getLocalPath() const { return localPath; }

    // local speeds planning
    void readLocalSpeedsConfig();

    inline LocalSpeedsStruct getLocalSpeeds() const { return localSpeeds; }

    // decision
    void readDecisionConfig();

    inline DecisionStruct getDecision() const { return decision; }

    // process
    void readProcessConfig();

    inline ProcessStruct getProcess() const { return process; }

    // move_cmd
    void readMoveCmdConfig();

  private:
    YAML::Node planningConfig;
    // vehicle
    VehicleStruct egoCar;
    VehicleStruct tpCar1;
    VehicleStruct tpCar2;
    VehicleStruct tpCar3;

    std::unordered_map<uint8, VehicleStruct> VehiclePairs;

    // pnc_map
    PNCMapStruct pncMap;

    // reference_line
    ReferenceLineStruct referenceLine;

    // global_path
    GlobalPathStruct globalPath;

    // local_path
    LocalPathStruct localPath;

    // local_speeds
    LocalSpeedsStruct localSpeeds;

    // decision
    DecisionStruct decision;

    // process
    ProcessStruct process;
  };
} // namespace Planning
#endif // !CONFIG_READER_H_
