//
// created by linxif2008 on 04/22/2024
//

#include "control/mpc_controller.h"

MpcController::MpcController(const Eigen::Matrix4d &Q, const Eigen::Matrix<double, 1, 1> &R, const size_t &mpc_window)
    : _mpc_window(mpc_window)
{
    // set A and B
    _A << 0, 1, 0, 0,
          0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0,
          0, 0, 0, 1,
          0, -b * m * l / P, m * g * l * (M + m) / P, 0;
    _B << 0, (I + m * l * l) / P, 0, m * l / P;


    _A_bar = Eigen::Matrix4d::Identity() + T * _A;
    _B_bar = T * _B;

    // calculate phi
    _phi.resize(4 * _mpc_window, 4);
    Eigen::Matrix4d phi_block = _A_bar;
    for (int i = 0; i < _mpc_window; i++)
    {
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                _phi.insert(i * 4 + j, k) = phi_block(j, k);
        phi_block = phi_block * _A_bar;
    }
    
    // calculate theta
    Eigen::Matrix4d _A_bar_inv = _A_bar.inverse();
    _theta.resize(4 * _mpc_window, _mpc_window);
    Eigen::Vector4d val = _B_bar;
    for (int i = 0; i < _mpc_window; i++)
    {
        for (int j = _mpc_window; j > i; j--)
        {
            size_t col = _mpc_window - j, row = i + col;
            for (int ii = 0; ii < 4; ii++)
                _theta.insert(row * 4 + ii, col) = val(ii, 0);
        }
        val = _A_bar_inv * val;
    }

    // generate Q
    _Q.resize(4 * _mpc_window, 4 * _mpc_window);
    for (int i = 0; i < _mpc_window; i++)
        for (int j = 0; j < 4; j++)
            _Q.insert(i * 4 + j, i * 4 + j) = Q(j, j);
    
    // generate R
    _R.resize(_mpc_window, _mpc_window);
    for (int i = 0; i < _mpc_window; i++)
        _R.insert(i, i) = R(0, 0);

    pos_estimate = 0.;
    rad_last = 0.;
}

bool MpcController::solveMPC()
{
    _E.resize(4 * _mpc_window, 1);
    _E = _phi * _x - _x_R;

    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd f(_mpc_window);
    H = 2 * (_theta.transpose() * _Q * _theta + _R);
    f = (2 * _E.transpose() * _Q * _theta).transpose();

    OsqpEigen::Solver solver;

    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(_mpc_window);
    solver.data()->setNumberOfConstraints(0);

    if (!solver.data()->setHessianMatrix(H))
        return false;
    if (!solver.data()->setGradient(f))
        return false;

    if (!solver.initSolver())
        return false;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return false;
    
    Eigen::VectorXd result = solver.getSolution();

    _u(0, 0) = result(0, 0);
    // if (_u(0, 0) > 0.5) _u(0, 0) = 0.5;
    // if (_u(0, 0) < -0.5) _u(0, 0) = -0.5;
    return true;
}

void MpcController::setTargetRad(const double &target_pos, const double &target_rad)
{
    _x_ref << target_pos, 0, target_rad, 0;

    // set R_x
    _x_R.resize(4 * _mpc_window, 1);
    for (int i = 0; i < _mpc_window; i++)
        for (int j = 0; j < 4; j++)
            _x_R.insert(i * 4 + j, 0) = _x_ref(j, 0);
}

void MpcController::handle(const double &pendulum_rad, const double &car_speed, double &wheel_frec)
{
    pos_estimate += car_speed * T;
    _x << pos_estimate, -car_speed, pendulum_rad, -(pendulum_rad - rad_last) / T;
    std::cout << "x: " << _x << std::endl;
    rad_last = pendulum_rad;

    if (solveMPC())
        wheel_frec = _u(0, 0) / r / 4;
    else
        wheel_frec = 0.;
}