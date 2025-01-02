#ifndef LOCAL_PATH_PLANNER_H_
#define LOCAL_PATH_PLANNER_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class LocalPathPlanner
    {
    public:
        LocalPathPlanner();
        LocalPathPlanner(const LocalPathPlanner&) = delete;
        LocalPathPlanner operator=(const LocalPathPlanner&) = delete;
        ~LocalPathPlanner() = default;
    private:

    };
}  // namespace Planning
#endif  // ! LOCAL_PATH_PLANNER_H_
