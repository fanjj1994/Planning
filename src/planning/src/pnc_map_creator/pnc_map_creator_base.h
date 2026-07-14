#ifndef PNC_MAP_CREATOR__PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR__PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include "config_reader.h"

namespace Planning
{
  using base_msgs::msg::PNCMap;
  using geometry_msgs::msg::Point;
  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;

  enum class PNCMapType
  {
    STRAIGHT,
    STURN,
  };

  class PNCMapCreatorBase // pnc地图创建器基类
  {
  public:
    virtual PNCMap create_pnc_map() = 0;
    inline PNCMap pnc_map() const {return pnc_map_;}
    inline MarkerArray pnc_map_markerarray() const {return pnc_map_markerarray_;}
    virtual ~PNCMapCreatorBase() {}

  protected:                                       // 子类中可以直接继承，但对象中不可以访问
    std::unique_ptr<ConfigReader> pnc_map_config_; // 配置
    int map_type_ = 0;                             // 地图类型
    PNCMap pnc_map_;                               // 地图
    MarkerArray pnc_map_markerarray_;              // rviz中用的地图

    Point p_mid_, pl_, pr_;    // 地图左中右三个点
    double theta_current_ = 0.0; // 地图当前的角度
    double len_step_ = 0.0;      // 长度步长
    double theta_step_ = 0.0;    // 角度步长
  };
} // namespace Planning
#endif // PNC_MAP_CREATOR__PNC_MAP_CREATOR_BASE_H_
