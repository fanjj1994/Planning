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
    void vehicle_cartesian_to_frenet_2path(const LocalPath &local_path, 
                                        const Referline &refer_line, 
                                        const std::shared_ptr<VehicleBase> &car) override ; //输出：定位点在路径上的投影点参数
  private:
  };
} // namespace Planning
#endif // MAIN_CAR_INFO_H_
