#include "osqp_test.h"

namespace Planning
{
  OsqpTest::OsqpTest() : Node("osqp_test_node")
  {
    RCLCPP_INFO(this->get_logger(),"osqp_test_node created");

    test_problem();
  }

  void OsqpTest::test_problem()
  {
    Eigen::SparseMatrix<double> P(2,2);
    Eigen::VectorXd Q(2);
    Eigen::SparseMatrix<double> A(2,2);
    Eigen::VectorXd lowerBound(2);
    Eigen::VectorXd upperBound(2);

    P.insert(0,0) = 2.0;
    P.insert(1,1) = 2.0;
    std::cout << "P:" << std::endl
                << P << std::endl;
    
    Q << -2,-2;
    std::cout << "Q:" << std::endl
                << Q << std::endl;

    A.insert(0,0) = 1.0;
    A.insert(1,1) = 1.0;
    std::cout << "A:" << std::endl
                << A << std::endl;

    lowerBound << 0.0,0.0;
    upperBound << 1.5,1.5;

    OsqpEigen::Solver solver;

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(2);
    if(!solver.data()->setHessianMatrix(P))
    {
        return;
    }
    if(!solver.data()->setGradient(Q))
    {
        return;
    }
    if(!solver.data()->setLinearConstraintsMatrix(A))
    {
        return;
    }
    if(!solver.data()->setLowerBound(lowerBound))
    {
        return;
    }
    if(!solver.data()->setUpperBound(upperBound))
    {
        return;
    }

    if(!solver.initSolver())
    {
        return;
    }

    Eigen::VectorXd QPSolution;

    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        return;
    }

    QPSolution = solver.getSolution();
    std::cout << "QPSolution:" << std::endl
            << QPSolution << std::endl;
  }

}// namespace Planning


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::OsqpTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
