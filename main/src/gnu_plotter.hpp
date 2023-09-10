#ifndef GNU_PLOTTER_HPP
#define GNU_PLOTTER_HPP

#include <gnuplot-iostream.h>

#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "kdl/frames_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames.hpp"

#include "robif2b/functions/kinova_gen3.h"

class GNUPlotter
{
public:
  // take logs dir as input to constructor
  GNUPlotter(std::string logs_dir, bool plot_data, bool save_data);

  ~GNUPlotter();

  void testPlot();
  std::string getNewFileName(std::string filename);


  // input: positions - std::vector<std::array<double, 3>>
  // input: target_pos = std::array<double, 3>
  void plotXYZ(const std::vector<std::array<double, 3>>& positions,
               const std::array<double, 3>& target_pos);
  void saveDataToCSV(const std::vector<std::array<double, 3>>& positions,
                     const std::array<double, 3>& target_pos);

  // Use with KDL
  // input: current_val - std::vector<KDL::Vector>
  // input: target_val - std::vector<KDL::Vector>
  // KDL::Vector contain - value.x(),value.y(),value.z()

  void plotXYZ(const std::vector<KDL::Vector>& current_val,
               const std::vector<KDL::Vector>& target_val,
               std::string title,
               double ytick);
  void saveDataToCSV(const std::vector<KDL::Vector>& current_val, 
                     const std::vector<KDL::Vector>& target_val,
                     std::string logname);

  // log everything from the robot , solver and the controller
  void saveDataToCSV(const std::vector<KDL::JntArray>& q,
                     const std::vector<KDL::JntArray>& qdot,
                     const std::vector<KDL::JntArray>& qddot,
                     const std::vector<KDL::JntArray>& constraint_tau,
                     const std::vector<KDL::Vector>& current_vel, 
                     const std::vector<KDL::Vector>& target_vel,
                     const std::vector<KDL::Vector>& current_pos, 
                     const std::vector<KDL::Vector>& target_pos,
                     const std::vector<KDL::Twist>& control_signal,
                     const std::vector<std::array<double, 7>>& joint_torque,
                     const std::vector<std::array<double, 7>>& actuar_current,
                     const std::vector<std::array<double, 7>>& actuar_voltage,
                     std::string logname);


  void saveDataToCSV(const std::vector<KDL::Vector>& current_pos, 
                     std::string logname);

private:
  // plot data
  bool plot_data_;
  // save data
  bool save_data_;
  // logs dir
  std::string logs_dir_;
};

#endif  // GNU_PLOTTER_HPP