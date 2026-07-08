#include "reference_line_creator.h"

namespace Planning
{
  ReferencelineCreator::ReferencelineCreator()//创建参考线
  {
    RCLCPP_INFO(rclcpp::get_logger("reference_line"),"reference_line_creator created");
  }
}// namespace Planning
