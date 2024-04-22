//
// created by linxif2008 on 04/15/2024
//

#include "control/pid_controller.h"
#include <algorithm>

void PidController::calculatePid()
{
    _error = _ref - _fdb;
    _cummulate_error += _i * _error;
    _cummulate_error = std::min(_i_max, std::max(-_i_max, _cummulate_error));

    _diff_error = _error - _last_error;
    _last_error = _error;

    _pid_output = _p * _error + _cummulate_error + _d * _diff_error;
}

PidController::PidController(const double &p, const double &i, const double &d, const double &i_max, const double &output_max)
    : _p(p), _i(i), _d(d), _i_max(i_max), _output_max(output_max)
{
    _cummulate_error = _last_error = 0.;
    _ref = _fdb = 0;
}

void PidController::setTargetRad(const double &target_rad)
{
    _ref = target_rad;
}

void PidController::handle(const double &pendulum_rad, const double &car_speed, double &wheel_frec)
{
    _ref = pendulum_rad;
    this->calculatePid();
    wheel_frec = _pid_output;
}