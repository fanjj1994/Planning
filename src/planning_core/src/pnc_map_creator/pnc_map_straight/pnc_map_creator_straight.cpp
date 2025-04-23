#include "pnc_map_creator_straight.h"

namespace Planning {
  PNCMapCreatorStraight::PNCMapCreatorStraight()
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PNCMapCreatorStraight is running");
    pncMapConfig = std::make_unique<ConfigReader>();
    pncMapConfig->readPNCMapConfig();
    mapType = static_cast<uint8>(PNCMapType::STRAIGHT); // ENUM Straight: 0U

    // map start point coordinate
    pCenter.x = -3.0;
    pCenter.y = pncMapConfig->getPNCMap().road_half_width_ / 2.0;
    pCenter.z = 0.0;

    // length step
    lengthStep = pncMapConfig->getPNCMap().segment_len_;

    // Map Init
    initPNCMap();
  }

  base_msgs::msg::PNCMap PNCMapCreatorStraight::createPNCMap()
  {
    drawStraightX(pncMap.road_length, 1.0);

    // rviz system malfunction: need to ensure number point of line list is even.
    if (pncMap.midline.points.size() % 2 != 0)
    {
      pncMap.midline.points.pop_back();
    }

    pncMapMarkerArray.markers.emplace_back(pncMap.midline);
    pncMapMarkerArray.markers.emplace_back(pncMap.left_boundary);
    pncMapMarkerArray.markers.emplace_back(pncMap.right_boundary);

    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PNCMapCreatorStraight is finished, center line has %ld points",
                pncMap.midline.points.size());
    return pncMap;
  }

  void PNCMapCreatorStraight::initPNCMap()
  {
    pncMap.header.frame_id = pncMapConfig->getPNCMap().frame_;
    pncMap.header.stamp = rclcpp::Clock().now();
    pncMap.road_length = pncMapConfig->getPNCMap().road_length_;
    pncMap.road_half_width = pncMapConfig->getPNCMap().road_half_width_;

    // center line format
    pncMap.midline.header = pncMap.header;
    pncMap.midline.ns = "pnc_map";
    pncMap.midline.id = 0;
    pncMap.midline.action = visualization_msgs::msg::Marker::ADD;
    pncMap.midline.type = visualization_msgs::msg::Marker::LINE_LIST;
    pncMap.midline.scale.x = 0.05; // line width
    pncMap.midline.color.r = 1.0;  // red
    pncMap.midline.color.g = 1.0;  // green
    pncMap.midline.color.b = 0.0;  // blue
    pncMap.midline.color.a = 1.0;  // alpha: 0.0 (transparent) to 1.0 (opaque)
    pncMap.midline.lifetime = rclcpp::Duration::max();
    pncMap.midline.frame_locked = true;

    // left boundary format
    pncMap.left_boundary = pncMap.midline;
    pncMap.left_boundary.id = 1;
    pncMap.left_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pncMap.left_boundary.color.r = 1.0; // red
    pncMap.left_boundary.color.g = 1.0; // green
    pncMap.left_boundary.color.b = 1.0; // blue

    // right boundary format
    pncMap.right_boundary = pncMap.left_boundary;
    pncMap.right_boundary.id = 2;
  }

  void PNCMapCreatorStraight::drawStraightX(const float64 &length, const float64 &plusFlag, const float64 &ratio)
  {
    float64 lenRuler{ 0.0 };
    while (lenRuler < length)
    {
      pLeft.x = pCenter.x;
      pLeft.y = pCenter.y + pncMapConfig->getPNCMap().road_half_width_;
      pRight.x = pCenter.x;
      pRight.y = pCenter.y - pncMapConfig->getPNCMap().road_half_width_;

      pncMap.midline.points.emplace_back(pCenter);
      pncMap.left_boundary.points.emplace_back(pLeft);
      pncMap.right_boundary.points.emplace_back(pRight);

      lenRuler += lengthStep * ratio;
      pCenter.x += lengthStep * plusFlag;
    }
  }

} // namespace Planning
