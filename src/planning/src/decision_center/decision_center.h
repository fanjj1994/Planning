#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include <vector>

namespace Planning
{
  constexpr double min_speed = 0.03;
  
  enum class SLPointType // 变道点位类型
  {
    LEFT_PASS,  // 从左绕
    RIGHT_PASS, // 从右绕
    STOP,       // 停止点
    START,      // 整个过程起点
    END         // 整个过程终点
  };

  struct SLPoint // 变道点位
  {
    double s_ = 0.0;
    double l_ = 0.0;
    int type_ = 0;
  };

  class DecisionCenter
  {
  public:
    DecisionCenter();
    void make_path_decision(const std::shared_ptr<VehicleBase> &car,
                            const std::vector<std::shared_ptr<VehicleBase>> &obses); //路径决策
    
    inline std::vector<SLPoint> sl_points() const { return sl_points_ ; } // 变道点位
  private:
    std::unique_ptr<ConfigReader> decision_config_;
    std::vector<SLPoint> sl_points_;  // 变道点位
  };
} // namespace Planning
#endif // DECISION_CENTER_H_
