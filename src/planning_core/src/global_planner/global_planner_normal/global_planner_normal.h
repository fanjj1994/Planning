#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning {
  class GlobalPlannerNormal final : public GlobalPlannerBase
  {
  public:
    GlobalPlannerNormal();
    GlobalPlannerNormal(const GlobalPlannerNormal&) = delete;
    GlobalPlannerNormal& operator=(const GlobalPlannerNormal&) = delete;
    ~GlobalPlannerNormal() = default;

  private:
  };
} // namespace Planning
#endif // ! GLOBAL_PLANNER_NORMAL_H_
