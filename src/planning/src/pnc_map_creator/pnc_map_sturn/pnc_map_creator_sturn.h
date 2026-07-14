#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorSturn : public PNCMapCreatorBase // s弯道地图
    {
    public:
        PNCMapCreatorSturn();
        PNCMap create_pnc_map() override ; //生成地图
    private:
        void init_pnc_map(); //初始化地图
        void draw_straight_x(const  double &length, const double &plus_flag , const double &ratio = 1.0);//绘制x方向直道
        void draw_arc(const double &angle, const double &plus_flag, const double &ratio = 1.0);//绘制弧线 正方向为逆时针，负方向为顺时针

    };
}  // namespace Planning
#endif  // PNC_MAP_CREATOR_STURN_H_
