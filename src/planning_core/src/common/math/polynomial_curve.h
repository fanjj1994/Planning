#ifndef POLYNOMIAL_CURVE_H_
#define POLYNOMIAL_CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <Eigen/Dense>
#include "common_type.h"

namespace Planning
{
  class PolynomialCurve
  {
  public:
    PolynomialCurve() = default;
    ~PolynomialCurve() = default;

    /// \brief  Compute linear (1st-order) polynomial coefficients under 2 boundary constraints.
    ///
    /// This function computes the coefficient vector $\mathbf{a}=[a_0,a_1]^\top$ of the linear polynomial
    ///
    /// $$
    /// y(x)=a_0 + a_1 x
    /// $$
    ///
    /// \details
    /// The implementation builds a $2\times2$ linear system $S\,\mathbf{a}=\mathbf{Y}$ from the two boundary values:
    ///
    /// $$
    /// \mathbf{Y}=[\text{start\_y},\;\text{end\_y}]^\top
    /// $$
    ///
    /// and solves it analytically.
    ///
    /// \note
    /// - The system is singular if \c start_x equals \c end_x.  The caller must ensure a non-zero segment length.
    ///
    /// \param[in]  start_x   Start x of the segment
    /// \param[in]  start_y   Boundary value $y(\text{start\_x})$
    /// \param[in]  end_x     End x of the segment
    /// \param[in]  end_y     Boundary value $y(\text{end\_x})$
    ///
    /// \return Eigen::Vector2d: The coefficient vector $[a_0,a_1]^\top$
    static Eigen::Vector2d computeLinearPolynomialCoefficients(const float64 &start_x, const float64 &start_y,
                                                               const float64 &end_x, const float64 &end_y);

    /// \brief  Compute cubic (3rd-order) polynomial coefficients under 4 boundary constraints.
    ///
    /// This function computes the coefficient vector $\mathbf{a}=[a_0,a_1,a_2,a_3]^\top$ of the cubic polynomial
    ///
    /// $$
    /// y(x)=a_0 + a_1 x + a_2 x^2 + a_3 x^3
    /// $$
    ///
    /// \details
    /// The implementation builds a $4\times4$ linear system $S\,\mathbf{a}=\mathbf{Y}$ by expanding $y(x)$ and
    /// $y'(x)$ at the two boundary points:
    ///
    /// $$
    /// \mathbf{Y}=[\text{start\_y},\;\text{start\_dydx},\;\text{end\_y},\;\text{end\_dydx}]^\top
    /// $$
    ///
    /// and solves it using Eigen's QR-based solver.
    ///
    /// \par Time-domain vs space-domain usage
    /// This solver is agnostic to the physical meaning of $x$ and $y$:
    /// - If $x=t$ and $y=s$, then $y'$ corresponds to velocity (time-domain longitudinal profile).
    /// - If $x=s$ and $y=l$, then $y'$ corresponds to $dl/ds$ (space-domain lateral profile).
    ///
    /// \note
    /// - The system becomes ill-conditioned or singular if \c start_x equals \c end_x (or if the two points are too
    ///   close). The caller should ensure a valid segment length.
    /// - For numerical stability when $|x|$ is large, it is common to shift the segment to a local coordinate
    ///   (e.g., use $\tilde{x}=x-\text{start\_x}$ so the interval becomes $[0,\Delta x]$).
    ///
    /// \param[in]  start_x       Start x of the segment
    /// \param[in]  start_y       Boundary value $y(\text{start\_x})$
    /// \param[in]  start_dydx    Boundary first derivative $y'(\text{start\_x})$
    /// \param[in]  end_x         End x of the segment
    /// \param[in]  end_y         Boundary value $y(\text{end\_x})$
    /// \param[in]  end_dydx      Boundary first derivative $y'(\text{end\_x})$
    ///
    /// \return Eigen::Vector4d: The coefficient vector $[a_0,a_1,a_2,a_3]^\top$
    static Eigen::Vector4d computeCubicPolynomialCoefficients(const float64 &start_x, const float64 &start_y,
                                                              const float64 &start_dydx, const float64 &end_x,
                                                              const float64 &end_y, const float64 &end_dydx);

    /// \brief  Compute quintic (5th-order) polynomial coefficients under 6 boundary constraints.
    ///
    /// This function computes the coefficient vector $\mathbf{a}=[a_0,a_1,a_2,a_3,a_4,a_5]^\top$ of the quintic
    /// polynomial
    ///
    /// $$
    /// y(x)=a_0 + a_1 x + a_2 x^2 + a_3 x^3 + a_4 x^4 + a_5 x^5
    /// $$
    ///
    ///
    /// \details
    /// The implementation builds a $6\times6$ linear system $S\,\mathbf{a}=\mathbf{Y}$ by expanding $y(x)$, $y'(x)$,
    /// and $y''(x)$ at the two boundary points:
    ///
    /// $$
    /// \mathbf{Y}=[\text{start\_y},\;\text{start\_dydx},\;\text{start\_ddydx},\;\text{end\_y},\;\text{end\_dydx},\;\text{end\_ddydx}]^\top
    /// $$
    ///
    /// and solves it using Eigen's QR-based solver (instead of explicitly inverting $S$).
    ///
    /// \par Time-domain vs space-domain usage
    /// This solver is agnostic to the physical meaning of $x$ and $y$:
    /// - If $x=t$ and $y=s$, then $y',y''$ correspond to velocity and acceleration (time-domain longitudinal profile).
    /// - If $x=s$ and $y=l$, then $y',y''$ correspond to $dl/ds$ and $d^2l/ds^2$ (space-domain lateral profile).
    ///
    /// \note
    /// - The system becomes ill-conditioned or singular if \c start_x equals \c end_x (or if the two points are too
    ///   close). The caller should ensure a valid segment length.
    /// - For numerical stability when $|x|$ is large, it is common to shift the segment to a local coordinate
    ///   (e.g., use $\tilde{x}=x-\text{start\_x}$ so the interval becomes $[0,\Delta x]$).
    ///
    /// \param[in]  start_x       Start x of the segment
    /// \param[in]  start_y       Boundary value $y(\text{start\_x})$
    /// \param[in]  start_dydx    Boundary first derivative $y'(\text{start\_x})$
    /// \param[in]  start_ddydx   Boundary second derivative $y''(\text{start\_x})$
    /// \param[in]  end_x         End x of the segment
    /// \param[in]  end_y         Boundary value $y(\text{end\_x})$
    /// \param[in]  end_dydx      Boundary first derivative $y'(\text{end\_x})$
    /// \param[in]  end_ddydx     Boundary second derivative $y''(\text{end\_x})$
    ///
    /// \return Eigen::Vector<float64, 6U>: The coefficient vector $[a_0,a_1,a_2,a_3,a_4,a_5]^\top$
    // clang-format off
    static Eigen::Vector<float64, 6U> computeQuinticPolynomialCoefficients(const float64 &start_x, const float64 &start_y, 
                                                                           const float64 &start_dydx, const float64 &start_ddydx,
                                                                           const float64 &end_x, const float64 &end_y,
                                                                           const float64 &end_dydx, const float64 &end_ddydx);
    // clang-format on
  private:
  };
} // namespace Planning
#endif // !POLYNOMIAL_CURVE_H_
