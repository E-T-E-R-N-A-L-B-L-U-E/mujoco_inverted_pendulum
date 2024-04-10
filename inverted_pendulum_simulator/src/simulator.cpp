#include <iostream>
#include <string>
#include <mujoco/mujoco.h>

#include <ros/ros.h>

#include "inverted_pendulum_simulator/mujoco_viewer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh("~");
    char error[1000];
    mjModel *m;
    mjData  *d;

    // load model from file and check for errors
    std::string urdf_file = PROJECT_DIR"/urdf/inverted_pendulum.urdf";
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

    // run simulation for 10 seconds
    ros::Rate rate(10);
    while (ros::ok())
    {
        mj_step(m, d);
        // std::cout << "step once" << std::endl;
    }

    viewer.close();
    // free model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}