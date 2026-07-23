#ifndef MAIN_CAR_INFO_H_
#define MAIN_CAR_INFO_H_

#include "vehicle_base.h"

namespace Planning
{
  class MainCar : public VehicleBase // 主车信息
  {
  public:
    MainCar();

    //定位点转frenet
    void vehicle_cartesian_to_frenet(const Referline &refer_line) override;
  private:
  };
} // namespace Planning
#endif // MAIN_CAR_INFO_H_
