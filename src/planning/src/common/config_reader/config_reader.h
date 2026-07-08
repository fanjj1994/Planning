#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

namespace Planning
{
  struct VehicleStruct // 车辆
  {
    int id_ = 0;              // 序号
    std::string frame_ = "";  // 坐标系
    double length_ = 0.0;    // 长度
    double width_ = 0.0;     // 宽度
    double pose_x_ = 0.0;    // x坐标
    double pose_y_ = 0.0;    // y坐标
    double pose_theta_ = 0.0; // 航向角
    double speed_ori_ = 0.0; // 初速度
  };

  struct PNCMapStruct // 地图
  {
    std::string frame_ = "";       // 坐标系
    int type_ = 0;                 // 地图类型
    double road_length_ = 0.0;     // 道长
    double road_half_width_ = 0.0; // 车道半宽
    double segment_len_ = 0.0;     // 道路分段长度
    double speed_limit_ = 0.0;     // 限速
  };

  struct GlobalPathStruct // 全局路径
  {
    int type_ = 0; // 类型
  };

  struct ReferLineStruct // 参考线
  {
    int type_ = 0;       // 类型
    int front_size_ = 0; // 前方点数 
    int back_size_ = 0;  // 后方点数
  };

  struct LocalPathStruct // 局部路径
  {
    int curve_type_ = 0; // 曲线类型
    int path_size_ = 0;  // 路径点数
  };

  struct LocalSpeedStruct // 速度
  {
    int speed_size_ = 0; // 速度点数
  };

  struct DecisionStruct // 决策
  {
    double safe_dis_l_ = 0.0; // 侧向安全距离
    double safe_dis_s_ = 0.0; // 纵向安全距离
  };

  struct ProcessStruct // 总流程
  {
    double obs_dis_ = 0.0; // 考虑障碍物的距离
  };

  class ConfigReader // 配置文件读取器
  {
  public:
    ConfigReader(); //配置文件读取器


    //vehicle
    void read_vehicle_config(VehicleStruct &vehicle, const std::string &name);
    void read_vehicles_config();
    inline VehicleStruct main_car() const { return main_car_; }
    inline VehicleStruct obs_car1() const { return obs_car1_; }
    inline VehicleStruct obs_car2() const { return obs_car2_; }
    inline VehicleStruct obs_car3() const { return obs_car3_; }
    inline std::unordered_map<int,VehicleStruct> obs_pair() const { return obs_pair_; }


    //pnc_map
    void read_pnc_map_config();
    inline PNCMapStruct pnc_map() const { return pnc_map_; }

    //global_path
    void read_global_path_config();
    inline GlobalPathStruct global_path() const { return global_path_; }

    //reference_line
    void read_reference_line_config();
    inline ReferLineStruct refer_line() const { return refer_line_; }

    //local_path
    void read_local_path_config();
    inline LocalPathStruct local_path() const { return local_path_; }

    //local_speed
    void read_local_speeds_config();
    inline LocalSpeedStruct local_speeds() const { return local_speeds_; }

    //decision
    void read_decision_config();
    inline DecisionStruct decision() const { return decision_; }

    //planning_process
    void read_planning_process_config();
    inline ProcessStruct process() const { return process_; }

    //move_cmd
    void read_move_cmd_config();

  private:
    YAML::Node planning_config;

    //vehicle
    VehicleStruct main_car_;
    VehicleStruct obs_car1_;
    VehicleStruct obs_car2_;
    VehicleStruct obs_car3_;
    std::unordered_map<int,VehicleStruct> obs_pair_;

    //pnc_map
    PNCMapStruct pnc_map_;

    //global_path
    GlobalPathStruct global_path_;

    //reference_line
    ReferLineStruct refer_line_;

    //local_path
    LocalPathStruct local_path_;

    //local_speed
    LocalSpeedStruct local_speeds_;

    //decision
    DecisionStruct decision_;

    //planning_process
    ProcessStruct process_;
  };
} // namespace Planning
#endif // CONFIG_READER_H_
