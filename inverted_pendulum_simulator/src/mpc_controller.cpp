//
// created by linxif2008 on 04/22/2024
//

#include "control/mpc_controller.h"

MpcController::MpcController(const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R)
{
    // set A and B
    _A << 0, 1, 0, 0,
          0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0,
          0, 0, 0, 1,
          0, -b * m * l / P, m * g * l * (M + m) / P, 0;
    _B << 0, (I + m * l * l) / P, 0, m * l / P;

    mpc_solver = std::make_shared<MpcSolver<4, 1, _mpc_window>>(_A, _B, Q, R, T);

    pos_estimate = 0.;
    rad_last = 0.;
}

void MpcController::setTargetRad(const double &target_pos, const double &target_rad)
{
    _x_ref << target_pos, 0, target_rad, 0;

    mpc_solver->updateReference(_x_ref);
}

void MpcController::handle(const double &pendulum_rad, const double &car_speed, double &wheel_frec)
{
    pos_estimate += car_speed * T;
    _x << pos_estimate, -car_speed, pendulum_rad, -(pendulum_rad - rad_last) / T;
    std::cout << "x: " << _x << std::endl;
    rad_last = pendulum_rad;

    mpc_solver->updateFeedBack(_x);

    if (mpc_solver->solveMPC(_u))
        wheel_frec = _u(0, 0) / r / 4;
    else
        wheel_frec = 0.;
}