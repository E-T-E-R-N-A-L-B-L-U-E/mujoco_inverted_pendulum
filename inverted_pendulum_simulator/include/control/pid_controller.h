//
// created by linxif2008 on 04/15/2024
//

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "control/controller_base.h"

#include <limits>

class PidController : public Controller
{
private:
    const double _p, _i, _d, _i_max, _output_max;
    double _cummulate_error, _error;
    double _last_error, _diff_error;
    double _ref, _fdb;
    double _pid_output;

    void calculatePid();
public:
    explicit PidController(const double &p, const double &i, const double &d, const double &i_max=std::numeric_limits<double>::max(), const double &output_max=std::numeric_limits<double>::max());

    void setTargetRad(const double &target_rad);

    void handle(const double &pendulum_rad, double &wheel_frec) override;
};


#endif //PID_CONTROLLER_H