//
// created by linxif2008 on 04/15/2024
//

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

class Controller
{
public:
    Controller(){}
    virtual void handle(const double &pendulum_rad, const double &car_speed, double &wheel_frec) {}
};


#endif // CONTROLLER_BASE_H