#ifndef OBS_CAR_INFO_H_
#define OBS_CAR_INFO_H_

#include "vehicle_base.h"

namespace Planning
{
  class ObsCar : public VehicleBase // 障碍物车辆信息
  {
  public:
    ObsCar(const int &id);

    //定位点转frenet
    void vehicle_cartesian_to_frenet(const Referline &refer_line) override;

  private:
  };
} // namespace Planning
#endif // Obs_CAR_INFO_H_
