#include <iostream>
#include <string>
#include <functional>
#include <mujoco/mujoco.h>

#include <ros/ros.h>

#include "inverted_pendulum_simulator/mujoco_viewer.h"
#include "inverted_pendulum_simulator/inverted_pendulum.h"
#include "control/pid_controller.h"


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

    PidController pid_controller(10, 0.01, 20, 1, 100);
    pid_controller.setTargetRad(0.);
    auto controller_interface = std::bind(&PidController::handle, &pid_controller, std::placeholders::_1, std::placeholders::_2);
    inverted_pendulum->setControlInterface(controller_interface);

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