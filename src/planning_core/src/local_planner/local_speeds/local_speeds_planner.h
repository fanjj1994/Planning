#ifndef LOCAL_SPEEDS_PLANNER_H_
#define LOCAL_SPEEDS_PLANNER_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class LocalSpeedsPlanner
    {
    public:
        LocalSpeedsPlanner();
        LocalSpeedsPlanner(const LocalSpeedsPlanner&) = delete;
        LocalSpeedsPlanner operator=(const LocalSpeedsPlanner&) = delete;
        ~LocalSpeedsPlanner() = default;
    private:

    };
}  // namespace Planning
#endif  // ! LOCAL_SPEEDS_PLANNER_H_
