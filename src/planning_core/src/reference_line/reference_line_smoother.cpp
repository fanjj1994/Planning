#include "reference_line_smoother.h"

namespace Planning
{
  ReferenceLineSmoother::ReferenceLineSmoother()
  {
    RCLCPP_INFO(rclcpp::get_logger("reference_line"), "ReferenceLineSmoother is running");

    // read config file
    referenceLineConfigReader = std::make_unique<ConfigReader>();
    referenceLineConfigReader->readReferenceLineConfig();
  }

  void ReferenceLineSmoother::SmoothReferenceLine(base_msgs::msg::Referline& referline)
  {
    const uint8 ptNum = referline.refer_line.size();
    // need at least 3 points to smooth
    if (ptNum < 3U)
    {
      return;
    }

    // build Matrix P
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity(); // 2x2 identity matrix
    Eigen::Matrix2d W1 = 2.0 * w1 * I;
    Eigen::Matrix2d W2 = 2.0 * w2 * I;
    Eigen::Matrix2d W3 = 2.0 * w3 * I;

    Eigen::Matrix2d block1 = W1 + W2 + W3;
    Eigen::Matrix2d block2 = -2.0 * W1 - W2;
    Eigen::Matrix2d block3 = -4.0 * W1 - W2;
    Eigen::Matrix2d block4 = 5.0 * W1 + 2.0 * W2 + W3;
    Eigen::Matrix2d block5 = 6.0 * W1 + 2.0 * W2 + W3;
    Eigen::Matrix2d block6 = 4.0 * W1 + 2.0 * W2 + W3;

    Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2U * ptNum, 2U * ptNum); // 2N x 2N

    if (ptNum == 3U) // process n = 3
    {
      /*
      upper triangle component:
      | W1 + W2 + W3,     -2W1  - W2,           W1 |
      |               4W1 + 2W2 + W3,    -2W1 - W2 |
      |                               W1 + W2 + W3 |
      */
      // Only fill the upper triangle part of P_tmp, the lower triangle part is symmetric
      P_tmp.block<2, 2>(0, 0) = block1;
      P_tmp.block<2, 2>(0, 2) = block2;
      P_tmp.block<2, 2>(0, 4) = W1;
      P_tmp.block<2, 2>(2, 2) = block6;
      P_tmp.block<2, 2>(2, 4) = block2;
      P_tmp.block<2, 2>(4, 4) = block1;
    }
    else
    {
      /*
      upper triangle component:
      | W1 + W2 + W3,     -2W1  - W2,             W1,            0,              0,      ...      0  |
      |               5W1 + 2W2 + W3,      -4W1 - W2,           W1,              0,      ...      0  |
      |                               6W1 + 2W2 + W3,    -4W1 - W2,             W1,     ...       0  |
      |                                                          .               .                .  |
      |                                                          .               .                .  |
      |                                                          .               .                .  |
      |                                             6W1 + 2W2 + W3,       -4W1 - W2,              W1 |
      |                                                              5W1 + 2W2 + W3,        -2W1 - W2|
      |                                                                                 W1 + W2 + W3 |
      */
      // Only fill the upper triangle part of P_tmp, the lower triangle part is symmetric
      for (uint8 i = 0U; i < ptNum; ++i)
      {
        if (i == 0U) // first row
        {
          P_tmp.block<2, 2>(i * 2, i * 2) = block1;
          P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
          P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
        }
        else if (i == 1U) // second row
        {
          P_tmp.block<2, 2>(i * 2, i * 2) = block4;
          P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
          P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
        }
        else if (i == (ptNum - 2U)) // second last row
        {
          P_tmp.block<2, 2>(i * 2, i * 2) = block4;
          P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
        }
        else if (i == (ptNum - 1U)) // last row
        {
          P_tmp.block<2, 2>(i * 2, i * 2) = block1;
        }
        else // middle rows, i = 2, 3, ..., n-3, always the same elements
        {
          P_tmp.block<2, 2>(i * 2, i * 2) = block5;
          P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
          P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
        }
      }
    }

    P_tmp = P_tmp.selfadjointView<Eigen::Upper>();       // make P_tmp symmetric
    Eigen::SparseMatrix<float64> P = P_tmp.sparseView(); // convert to sparse matrix

    Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2U * ptNum, 2U * ptNum); // 2N x 2N identity matrix
    Eigen::SparseMatrix<float64> A = A_tmp.sparseView();                       // convert to sparse matrix

    // get original reference line points
    Eigen::VectorXd X(2U * ptNum); // 2N x 1
    for (uint8 i = 0U; i < ptNum; ++i)
    {
      X(i * 2) = referline.refer_line[i].pose.pose.position.x;
      X(i * 2 + 1U) = referline.refer_line[i].pose.pose.position.y;
    }

    // build Vector q
    Eigen::VectorXd q = -2.0 * X; // 2N x 1
    // tolerance buffer, set to 0.2m for both x and y direction
    Eigen::VectorXd buff = Eigen::VectorXd::Constant(2U * ptNum, 0.2);   // 2N x 1
    buff(0) = buff(1) = buff(2 * ptNum - 2) = buff(2 * ptNum - 1) = 0.0; // first and last point fixed, no buffer
    Eigen::VectorXd l = X - buff;                                        // lower bound
    Eigen::VectorXd u = X + buff;                                        // upper bound

    // OSQP solver
    OsqpEigen::Solver solver;

    // set solver settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // initialize solver
    solver.data()->setNumberOfVariables(2U * ptNum);
    solver.data()->setNumberOfConstraints(2U * ptNum);
    if (!solver.data()->setHessianMatrix(P))
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to set Hessian Matrix");
      return;
    }
    if (!solver.data()->setGradient(q))
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to set Gradient");
      return;
    }
    if (!solver.data()->setLinearConstraintsMatrix(A))
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to set Linear Constraints Matrix");
      return;
    }
    if (!solver.data()->setLowerBound(l))
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to set Lower Bound");
      return;
    }
    if (!solver.data()->setUpperBound(u))
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to set Upper Bound");
      return;
    }
    if (!solver.initSolver())
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to initialize OSQP solver");
      return;
    }

    // solve
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
      RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "Fail to solve the QP problem");
      return;
    }
    Eigen::VectorXd X_smooth = solver.getSolution(); // 2N x 1

    // update referline points
    for (uint8 i = 0U; i < ptNum; ++i)
    {
      referline.refer_line[i].pose.pose.position.x = X_smooth(i * 2);
      referline.refer_line[i].pose.pose.position.y = X_smooth(i * 2 + 1U);
    }
  }

} // namespace Planning
