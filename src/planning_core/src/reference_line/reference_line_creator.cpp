#include "reference_line_creator.h"

namespace Planning
{

  base_msgs::msg::Referline
  ReferenceLineCreator::createReferenceLine(const nav_msgs::msg::Path& globalPath,
                                            const geometry_msgs::msg::PoseStamped& vehiclePose)
  {
    if (globalPath.poses.empty()) // check if global path is empty
    {
      return referenceLine;
    }

    // find match point
    matchPointIndex = Curve::findMatchPointIndex(globalPath, lastMatchPointIndex, vehiclePose);
    lastMatchPointIndex = matchPointIndex; // update last match point index
    if (matchPointIndex < 0)               // cannot find match point
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Cannot find match point!");
      return referenceLine;
    }

    // compute front point and back point index
    const uint16 global_path_size = static_cast<uint16>(globalPath.poses.size());
    frontIndex = (static_cast<uint16>(matchPointIndex + referenceLineConfigReader->getReferenceLine().front_size_) >=
                  global_path_size - 1)
                     ? (global_path_size - 1)
                     : (matchPointIndex + referenceLineConfigReader->getReferenceLine().front_size_);
    backIndex = (static_cast<int32>(matchPointIndex - referenceLineConfigReader->getReferenceLine().back_size_) <= 0)
                    ? 0
                    : (matchPointIndex - referenceLineConfigReader->getReferenceLine().back_size_);

    // fulfill reference line
    referenceLine.header.frame_id = referenceLineConfigReader->getPNCMap().frame_;
    referenceLine.header.stamp = rclcpp::Clock().now();
    referenceLine.refer_line.clear(); // clear previous reference line
    base_msgs::msg::ReferlinePoint pt_tmp;
    for (int16 i = backIndex; i <= frontIndex; i++)
    {
      pt_tmp.pose = globalPath.poses[i];
      referenceLine.refer_line.emplace_back(pt_tmp);
    }

    // smooth reference line
    referenceLineSmoother->SmoothReferenceLine(referenceLine);

    // compute heading, kappa, dkappa for reference line
    Curve::calculateProjectedPointParameters(referenceLine);

    RCLCPP_INFO(rclcpp::get_logger("reference_line"),
                "Create reference line successfully! matchPointIndex: %d, frontIndex: %d, backIndex: %d, size: %ld",
                matchPointIndex, frontIndex, backIndex, referenceLine.refer_line.size());

    return referenceLine;
  }

  nav_msgs::msg::Path ReferenceLineCreator::referenceLineToRviz()
  {
    referlineRviz.header = referenceLine.header;
    referlineRviz.poses.clear();

    geometry_msgs::msg::PoseStamped pt_tmp;
    for (const auto& pt : referenceLine.refer_line)
    {
      pt_tmp.header = referlineRviz.header;
      pt_tmp.pose = pt.pose.pose;
      referlineRviz.poses.emplace_back(pt_tmp);
    }

    return referlineRviz;
  }

} // namespace Planning
