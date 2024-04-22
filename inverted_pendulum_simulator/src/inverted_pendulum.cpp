//
// created by linxif2008
//

#include "inverted_pendulum_simulator/inverted_pendulum.h"
#include <iostream>

void InvertedPendulum::_read()
{
    if (_tick < 0)
    {
        _wheel_pos = _data->qpos[_model->jnt_qposadr[_wheel_id[0]]];
        _tick = _data->time;
    }
    double dt = _data->time - _tick;
    _tick = _data->time;
    _pendulum_rad = _data->qpos[_model->jnt_qposadr[_pendulum_id]];
    _wheel_speed = (_data->qpos[_model->jnt_qposadr[_wheel_id[0]]] - _wheel_pos);
    if (_wheel_speed > 3.1415926)
        _wheel_speed = (_wheel_speed - 2 * 3.1415926);
    else if (_wheel_speed < -3.1415926)
        _wheel_speed = (_wheel_speed + 2 * 3.1415926);
    if (dt > 0)
        _wheel_speed = _wheel_speed / dt * 0.05;
    else
        _wheel_speed = 0.;
    _wheel_pos = _data->qpos[_model->jnt_qposadr[_wheel_id[0]]];
}

void InvertedPendulum::_write()
{
    for (int i = 0; i < 4; i++)
        _data->qfrc_applied[_model->jnt_dofadr[_wheel_id[i]]] = _wheel_frec;
}

InvertedPendulum::InvertedPendulum(mjModel* model, mjData *data)
{
    _model = model;
    _data = data;

    for (int i = 0; i < 4; i++)
        _wheel_id[i] = mj_name2id(_model, mjOBJ_JOINT, _WHEEL_NAMES[i].c_str());
    _pendulum_id = mj_name2id(_model, mjOBJ_JOINT, _PENDULUM_NAME.c_str());

    _wheel_frec = 0.;
    _pendulum_rad = 0.;
    _wheel_pos = 0.;
    _wheel_speed = 0.;
    _tick = -1.;
    _control_interface = nullptr;
}

void InvertedPendulum::handle()
{
    this->_read();
    if (_control_interface != nullptr)
        _control_interface(this->_pendulum_rad, this->_wheel_speed, this->_wheel_frec);
    this->_write();
}

void InvertedPendulum::setControlInterface(std::function<void(const double&, const double&, double&)> control_interface)
{
    _control_interface = control_interface;
}

void InvertedPendulum::echo()
{
    std::cout << "current pendulum angle: " << _pendulum_rad / 3.14 * 180. << ", current control force: " << _wheel_frec << std::endl;
}