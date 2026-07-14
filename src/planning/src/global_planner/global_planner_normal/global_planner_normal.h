#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning
{
class GlobalPlannerNormal : public GlobalPlannerBase// 普通全局路径规划器
    {
    public:
        GlobalPlannerNormal();
        Path search_global_path(const PNCMap &pnc_map);//搜索全局路径

    private:

    };
}  // namespace Planning
#endif  // GLOBAL_PLANNER_NORMAL_H_
