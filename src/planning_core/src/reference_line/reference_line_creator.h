#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"

namespace Planning
{
class ReferenceLineCreator
{
private:
  std::unique_ptr<ConfigReader> referenceLineConfigReader;  // config reader

  base_msgs::msg::Referline referenceLine;  // reference line for planning

  nav_msgs::msg::Path referlineRviz;  // reference line for rviz display

  std::shared_ptr<ReferenceLineSmoother> referenceLineSmoother;  // reference line smoother

  int16 lastMatchPointIndex;  // last match point index
  int16 matchPointIndex;      // current match point index
  int16 frontIndex;           // front index
  int16 backIndex;            // back index

public:
  ReferenceLineCreator() : lastMatchPointIndex(-1), matchPointIndex(-1), frontIndex(-1), backIndex(-1)
  {
    RCLCPP_INFO(rclcpp::get_logger("reference_line"), "ReferenceLineCreator is initialized.");
    // read config file
    referenceLineConfigReader = std::make_unique<ConfigReader>();
    referenceLineConfigReader->readReferenceLineConfig();

    // initialize reference line smoother
    referenceLineSmoother = std::make_shared<ReferenceLineSmoother>();
  }
  ReferenceLineCreator(const ReferenceLineCreator&) = delete;
  ReferenceLineCreator& operator=(const ReferenceLineCreator&) = delete;
  ~ReferenceLineCreator() = default;

  // create reference line
  base_msgs::msg::Referline createReferenceLine(const nav_msgs::msg::Path& globalPath,
                                                const geometry_msgs::msg::PoseStamped& vehiclePose);
  // reference line convert to rviz display
  nav_msgs::msg::Path referenceLineToRviz();

  inline base_msgs::msg::Referline getReferenceLine() const
  {
    return referenceLine;
  }

  inline nav_msgs::msg::Path getReferenceLineRviz() const
  {
    return referlineRviz;
  }

  inline int16 getMatchPointIndex() const
  {
    return matchPointIndex;
  }

  inline int16 getFrontIndex() const
  {
    return frontIndex;
  }

  inline int16 getBackIndex() const
  {
    return backIndex;
  }
};
}  // namespace Planning
#endif  // ! REFERENCE_LINE_CREATOR_H_
