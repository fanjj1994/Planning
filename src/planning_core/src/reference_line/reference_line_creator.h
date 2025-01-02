#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning {
  class ReferenceLineCreator
  {
  public:
    ReferenceLineCreator();
    ReferenceLineCreator(const ReferenceLineCreator&) = delete;
    ReferenceLineCreator& operator=(const ReferenceLineCreator&) = delete;
    ~ReferenceLineCreator() = default;

  private:
  };
} // namespace Planning
#endif // ! REFERENCE_LINE_CREATOR_H_
