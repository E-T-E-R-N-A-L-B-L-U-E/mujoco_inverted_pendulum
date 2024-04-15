//
// created by linxif2008
//

#include "inverted_pendulum_simulator/inverted_pendulum.h"
#include <iostream>

void InvertedPendulum::_read()
{
    _pendulum_rad = _data->qpos[_model->jnt_qposadr[_pendulum_id]];
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
    _control_interface = nullptr;
}

void InvertedPendulum::handle()
{
    this->_read();
    if (_control_interface != nullptr)
        _control_interface(this->_pendulum_rad, this->_wheel_frec);
    this->_write();
}

void InvertedPendulum::setControlInterface(std::function<void(const double&, double&)> control_interface)
{
    _control_interface = control_interface;
}

void InvertedPendulum::echo()
{
    std::cout << "current pendulum angle: " << _pendulum_rad / 3.14 * 180. << ", current control force: " << _wheel_frec << std::endl;
}