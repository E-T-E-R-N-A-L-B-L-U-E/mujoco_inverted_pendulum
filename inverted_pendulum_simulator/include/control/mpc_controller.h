//
// created by linxif2008 on 04/22/2024
//

#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "control/controller_base.h"
#include <OsqpEigen/OsqpEigen.h>


class MpcController : public Controller
{
private:
    const double M = 2. + 0.39 * 4;        // car mass
    const double l = 0.3;                   // pendulum length
    const double g = 9.8;                   // g
    const double m = 0.7;                   // pendulum mass
    const double b = 0.001 * (M + m * 4) * g;  // rolling frict
    const double I = m * l * l;             // pendulum inertial
    const double r = 0.05;                  // wheel radius
    const double sf = 1;                    // sliding frict

    const double P = (M + m) * I + M * m * l * l;
    const double T = 0.002;

    const size_t _mpc_window;

    Eigen::Matrix4d _A, _A_bar;
    Eigen::Vector4d _B, _B_bar;

    Eigen::SparseMatrix<double> _phi, _theta, _x_R, _E, _Q, _R;

    Eigen::Vector4d _x, _x_ref;
    Eigen::Matrix<double, 1, 1> _u;

    double pos_estimate, rad_last;

    bool solveMPC();
public:

    explicit MpcController(const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R, const size_t &mpc_window = 10);

    void setTargetRad(const double &target_pos, const double &target_rad);

    void handle(const double &pendulum_rad, const double &car_speed, double &wheel_frec) override;
};

#endif // MPC_CONTROLLER_H