//
// created by linxif2008
//
#ifndef INVERTED_PENDULUM_H
#define INVERTED_PENDULUM_H

#include <string>
#include <functional>
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
    double _wheel_frec, _wheel_pos, _wheel_speed;
    double _tick;

    void _read();
    void _write();

    std::function<void(const double&, const double&, double&)> _control_interface;
public:
    explicit InvertedPendulum(mjModel* model, mjData *data);

    void handle();

    void setControlInterface(std::function<void(const double&, const double&, double&)> control_interface);

    void echo();
};


#endif //INVERTED_PENDULUM_H