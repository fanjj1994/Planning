#ifndef POLYNOMIAL_CURVE_H_
#define POLYNOMIAL_CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <Eigen/Dense>

namespace Planning
{
    class PolynomialCurve
    {
    public:
        PolynomialCurve() = default ;

        // 五次多项式
        static Eigen::Vector<double,6> quintic_polynomial(const double &start_x, const double &start_y,
                                                          const double &start_dy_dx, const double &start_ddy_dx,
                                                          const double &end_x, const double &end_y,
                                                          const double &end_dy_dx, const double &end_ddy_dx); 
    };
}  // namespace Planning
#endif  // POLYNOMIAL_CURVE_H_
