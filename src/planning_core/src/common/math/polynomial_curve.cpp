#include "polynomial_curve.h"

namespace Planning
{
  Eigen::Vector2d PolynomialCurve::computeLinearPolynomialCoefficients(const float64 &start_x, const float64 &start_y,
                                                                       const float64 &end_x, const float64 &end_y)
  {
    Eigen::Matrix2d S;
    S << 1.0, start_x, 1.0, end_x;

    Eigen::Vector2d Y;
    Y << start_y, end_y;

    return S.colPivHouseholderQr().solve(Y);
  }

  Eigen::Vector4d PolynomialCurve::computeCubicPolynomialCoefficients(const float64 &start_x, const float64 &start_y,
                                                                      const float64 &start_dydx, const float64 &end_x,
                                                                      const float64 &end_y, const float64 &end_dydx)
  {
    Eigen::Matrix4d S;
    S << 1.0, start_x, start_x * start_x, start_x * start_x * start_x, 0.0, 1.0, 2.0 * start_x, 3.0 * start_x * start_x,
        1.0, end_x, end_x * end_x, end_x * end_x * end_x, 0.0, 1.0, 2.0 * end_x, 3.0 * end_x * end_x;

    Eigen::Vector4d Y;
    Y << start_y, start_dydx, end_y, end_dydx;

    return S.colPivHouseholderQr().solve(Y);
  }

  // clang-format off
  Eigen::Vector<float64, 6U> PolynomialCurve::computeQuinticPolynomialCoefficients(const float64 &start_x, const float64 &start_y, 
                                                                                   const float64 &start_dydx, const float64 &start_ddydx,
                                                                                   const float64 &end_x, const float64 &end_y,
                                                                                   const float64 &end_dydx, const float64 &end_ddydx)
  // clang-format on
  {
    const float64 start_xSquared = start_x * start_x;
    const float64 start_xCubed = start_xSquared * start_x;
    const float64 start_xFourth = start_xCubed * start_x;
    const float64 start_xFifth = start_xFourth * start_x;
    const float64 end_xSquared = end_x * end_x;
    const float64 end_xCubed = end_xSquared * end_x;
    const float64 end_xFourth = end_xCubed * end_x;
    const float64 end_xFifth = end_xFourth * end_x;

    Eigen::Matrix<float64, 6U, 6U> S;

    S << 1.0, start_x, start_xSquared, start_xCubed, start_xFourth, start_xFifth, 0.0, 1.0, 2.0 * start_x,
        3.0 * start_xSquared, 4.0 * start_xCubed, 5.0 * start_xFourth, 0.0, 0.0, 2.0, 6.0 * start_x,
        12.0 * start_xSquared, 20.0 * start_xCubed, 1.0, end_x, end_xSquared, end_xCubed, end_xFourth, end_xFifth, 0.0,
        1.0, 2.0 * end_x, 3.0 * end_xSquared, 4.0 * end_xCubed, 5.0 * end_xFourth, 0.0, 0.0, 2.0, 6.0 * end_x,
        12.0 * end_xSquared, 20.0 * end_xCubed;

    Eigen::Vector<float64, 6U> Y;
    Y << start_y, start_dydx, start_ddydx, end_y, end_dydx, end_ddydx;

    return S.colPivHouseholderQr().solve(Y);
  }

} // namespace Planning