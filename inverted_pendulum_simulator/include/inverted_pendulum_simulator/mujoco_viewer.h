//
// created by linxif2008
//

#ifndef MUJUCO_VIEWER
#define MUJUCO_VIEWER

#include <memory>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <thread>

class MujocoViewer
{
private:
    std::shared_ptr<mjModel> _model;
    std::shared_ptr<mjData>  _data;

    mjvCamera   _camera;
    mjvOption   _option;
    mjvPerturb  _perturb;
    mjvScene    _scene;
    mjrContext  _context;
    GLFWwindow  *_window;

    bool        _window_opened;
    std::thread _window_update_thread;
    void windowUpdateThread(const std::string &window_name);
public:
    explicit MujocoViewer(mjModel *model, mjData *data, const std::string &window_name);
    explicit MujocoViewer(std::shared_ptr<mjModel> model, std::shared_ptr<mjData> data, const std::string &window_name);

    void close();
    bool isWindowClosed() const;
};

#endif //MUJUCO_VIEWER