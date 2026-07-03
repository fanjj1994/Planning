#ifndef Planning_Process_H_
#define Planning_Process_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
  class PlanningProcess : public rclcpp::Node //规划总流程
  {
    public:
        PlanningProcess() ;
        bool process();//总流程

    private:

  };
}  // namespace Planning
#endif  // Planning_Process_H_