//
// created by linxif2008
//

#include "inverted_pendulum_simulator/mujoco_viewer.h"
#include <chrono>
#include <iostream>
#include <assert.h>
using namespace std::chrono;

void MujocoViewer::windowUpdateThread(const std::string &window_name)
{
    // init glfw and windows
    glfwInit();
    _window = glfwCreateWindow(1200, 900, window_name.c_str(), nullptr, nullptr);
    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    // initialize visualization data structure
    mjv_defaultCamera(&_camera);
    mjv_defaultOption(&_option);
    mjv_defaultPerturb(&_perturb);
    mjr_defaultContext(&_context);

    // create scene and context
    mjv_makeScene(_model.get(), &_scene, 1000);
    mjr_makeContext(_model.get(), &_context, mjFONTSCALE_100);

    while (_window_opened && !glfwWindowShouldClose(_window))
    {
        mjrRect view_port = {0, 0, 0, 0};
        glfwGetFramebufferSize(_window, &view_port.width, &view_port.height);

        mjv_updateScene(_model.get(), _data.get(), &_option, nullptr, &_camera, mjCAT_ALL, &_scene);
        mjr_render(view_port, &_scene, &_context);

        glfwSwapBuffers(_window);
        
        glfwPollEvents();

        // sleep
        std::this_thread::sleep_for(1ms);
    }
    if (glfwWindowShouldClose(_window))
        _window_opened = false;

    glfwTerminate();

    mjv_freeScene(&_scene);
    mjr_freeContext(&_context);
    std::cout << "window closed" << std::endl;
}

MujocoViewer::MujocoViewer(mjModel *model, mjData *data, const std::string &window_name)
{
    assert(model != nullptr && data != nullptr);
    _model = std::shared_ptr<mjModel>(model, [](mjModel *){});
    _data = std::shared_ptr<mjData>(data, [](mjData *){});

    _window_update_thread = std::thread(&MujocoViewer::windowUpdateThread, this, window_name);
    _window_opened = true;
}

MujocoViewer::MujocoViewer(std::shared_ptr<mjModel> model, std::shared_ptr<mjData> data, const std::string &window_name)
{
    assert(model.get() != nullptr && data.get() != nullptr);
    _model = model;
    _data = data;

    _window_update_thread = std::thread(&MujocoViewer::windowUpdateThread, this, window_name);
    _window_opened = true;
}

void MujocoViewer::close()
{
    _window_opened = false;
}

bool MujocoViewer::isWindowClosed() const
{
    return _window_opened;
}