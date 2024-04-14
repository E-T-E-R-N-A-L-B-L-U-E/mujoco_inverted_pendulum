#include <iostream>
#include <string>
#include <mujoco/mujoco.h>

#include <ros/ros.h>

#include "inverted_pendulum_simulator/mujoco_viewer.h"

std::string wheel_joint_names[4] = {
    std::string("fl_joint"),
    std::string("fr_joint"),
    std::string("bl_joint"),
    std::string("br_joint"),
};

std::string pendulum_name = "pendulum_joint";

void handle(mjModel *m, mjData *d)
{
    const mjtNum pendulum_target_rad = 0;
    const double P = 10, I = 0.01, D = 20;
    int pendulum_id = mj_name2id(m, mjOBJ_JOINT, pendulum_name.c_str());
    
    mjtNum pendulum_current_rad = d->qpos[m->jnt_qposadr[pendulum_id]];
    mjtNum delta = pendulum_target_rad - pendulum_current_rad;
    static mjtNum last_delta = delta;
    static mjtNum cummulate_deta = 0;
    cummulate_deta += delta;
    mjtNum pid_result = P * delta + I * cummulate_deta + D * (delta - last_delta);
    // d->qfrc_applied[m->jnt_dofadr[pendulum_id]] = pid_result;

    int wheel_id[4];
    for (int i = 0; i < 4; i++)
        wheel_id[i] = mj_name2id(m, mjOBJ_JOINT, wheel_joint_names[i].c_str());
    for (int i = 0; i < 4; i++)
        d->qfrc_applied[m->jnt_dofadr[wheel_id[i]]] = -pid_result;

    last_delta = delta;
    std::cerr << "id: " << pendulum_id << ", jntadr: " << m->body_jntadr[pendulum_id] << ", qposadr: " << m->jnt_qposadr[pendulum_id] << std::endl;
    for (int i = 0; i < 8; i++)
        std::cerr << d->qpos[i] << ", ";
    std::cerr << std::endl;
    std::cerr << "current angle: " << pendulum_current_rad / 3.14 * 180. << " ctrl: " << -pid_result << std::endl;
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

    // run simulation for 10 seconds
    ros::Rate rate(10);
    while (ros::ok())
    {
        using namespace std::chrono;
        handle(m, d);
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