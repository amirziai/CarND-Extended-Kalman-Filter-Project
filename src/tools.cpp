#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground truth" << std::endl;
        return rmse;
    }

    for (unsigned int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        // multiply by coefficient
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // calculate the mean
    rmse = rmse / estimations.size();

    // calculate the squared error
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj = MatrixXd::Zero(3, 4);
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = c1 * c2;

    if (fabs(c1) > 0.00001) {
        Hj <<   (px / c2), (py / c2), 0, 0,
                -(py / c1), (px / c1), 0, 0,
                py * (vx * py - vy * px) / c3, px * (px * vy - py * vx), px / c2, py / c2;
        return Hj;
    }

    std::cout << "CalculateJacobian () - Error - Underflow. Zero out the Jacobian" << std::endl;
    return Hj;
}
