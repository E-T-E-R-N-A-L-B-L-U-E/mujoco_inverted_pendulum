//
// created by linxif2008
//
#ifndef INVERTED_PENDULUM_H
#define INVERTED_PENDULUM_H

#include <string>
#include <mujoco/mujoco.h>


class InvertedPendulum
{
private:

    // define joint names
    const std::string _WHEEL_NAMES[4] = {
        std::string("fl_joint"),
        std::string("fr_joint"),
        std::string("bl_joint"),
        std::string("br_joint"),
    };
    const std::string _PENDULUM_NAME = std::string("pendulum_joint");
    
    int _wheel_id[4];
    int _pendulum_id;


    mjModel *_model;
    mjData *_data;

    double _pendulum_rad;
    double _wheel_frec;

    void _read();
    void _write();

    void (*_control_interface)(const double &pendulum_rad, double &wheel_frec);
public:
    explicit InvertedPendulum(mjModel* model, mjData *data);

    void handle();

    void setControlInterface(void (*control_interface)(const double &, double &));

    void echo();
};


#endif //INVERTED_PENDULUM_H