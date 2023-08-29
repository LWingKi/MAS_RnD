#ifndef MUJOCO_HPP
#define MUJOCO_HPP

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <filesystem>
#include <iostream>

class RobotSimulation
{
private:
  // Member variables
  bool button_left;
  bool button_middle;
  bool button_right;
  double lastx;
  double lasty;

  std::filesystem::path path;
  std::string robot_urdf;
  char* error;
  mjModel* model;
  mjData* data;
  mjvScene scn;
  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;
  mjvFigure fig;
  mjrContext con;

public:
  // Constructor
  RobotSimulation();

  // Destructor
  ~RobotSimulation();

  // Methods
  void mouse_button(GLFWwindow* window, int button, int act, int mods);
  void mouse_move(GLFWwindow* window, double xpos, double ypos);

  static void static_mouse_move(GLFWwindow* window, double xpos, double ypos)
  {
    instance->mouse_move(window, xpos, ypos);
  }

  static void static_mouse_button(GLFWwindow* window, int button, int act, int mods)
  {
    instance->mouse_button(window, button, act, mods);
  }

  static RobotSimulation* instance;
  
  int run();
};

#endif  // MUJOCO_HPP
