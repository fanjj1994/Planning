#ifndef LOCAL_TRAJECTORY_COMBINER_H_
#define LOCAL_TRAJECTORY_COMBINER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"

namespace Planning {
  class LocalTrajectoryCombiner
  {
  public:
    LocalTrajectoryCombiner();
    LocalTrajectoryCombiner(const LocalTrajectoryCombiner&) = delete;
    LocalTrajectoryCombiner operator=(const LocalTrajectoryCombiner&) = delete;
    ~LocalTrajectoryCombiner() = default;

  private:
  };
} // namespace Planning
#endif // ! LOCAL_TRAJECTORY_COMBINER_H_
