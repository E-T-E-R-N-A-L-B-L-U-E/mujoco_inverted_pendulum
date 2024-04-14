#include <iostream>
#include <string>
#include <mujoco/mujoco.h>

#include <ros/ros.h>

#include "inverted_pendulum_simulator/mujoco_viewer.h"
#include "inverted_pendulum_simulator/inverted_pendulum.h"

void controlInterface(const double &pendulum_current_rad, double &wheel_frec)
{
    const double P = 10, I = 0.01, D = 20;
    const double pendulum_target_rad = 0;
    double delta = pendulum_target_rad - pendulum_current_rad;
    static double last_delta = delta;
    static double cummulate_deta = 0;
    cummulate_deta += delta;
    double pid_result = P * delta + I * cummulate_deta + D * (delta - last_delta); 
    wheel_frec = -pid_result;  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh("~");
    char error[1000];
    mjModel *m;
    mjData  *d;

    // load model from file and check for errors
    std::string urdf_file = PROJECT_DIR"/urdf/inverted_pendulum.xml";
    std::cout << "loading urdf file from: " << urdf_file << std::endl;
    m = mj_loadXML(urdf_file.c_str(), NULL, error, 1000);
    if( !m )
    {
        printf("%s\n", error);
        return 1;
    }

    // make data corresponding to model
    d = mj_makeData(m);

    MujocoViewer viewer(m, d, "demo");
    std::shared_ptr<InvertedPendulum> inverted_pendulum = std::make_shared<InvertedPendulum>(m, d);
    inverted_pendulum->setControlInterface(&controlInterface);

    // run simulation for 10 seconds
    ros::Rate rate(10);
    while (ros::ok())
    {
        using namespace std::chrono;
        inverted_pendulum->handle();
        inverted_pendulum->echo();
        mj_step(m, d);
        std::this_thread::sleep_for(10ms);
        // std::cout << "step once" << std::endl;
    }

    viewer.close();
    // free model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}