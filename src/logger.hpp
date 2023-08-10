#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "kdl/jntarray.hpp"
#include "kdl/kinfam_io.hpp"

class Logger
{
public:
  enum LogLevel
  {
    INFO,
    WARNING,
    ERROR
  };

  Logger(bool log_to_terminal, bool log_to_file, std::string logs_dir, bool show_date = false);

  ~Logger();

  void test();

  template <typename T, size_t N>
  void logInfo(const std::array<T, N>& arr);

  template <typename T>
  void logInfo(const std::vector<T>& vec);

  template <typename T>
  void logInfo(const std::vector<std::vector<T>>& vec_of_vec);

  void logInfo(const KDL::JntArray& jnt_array);

  void logInfo(const KDL::Vector& vector);

  void logInfo(const KDL::Twist& twist);

  void logInfo(const KDL::Jacobian& jacobian);

  template <typename... Args>
  void logInfo(const char* format, Args... args);

  template <typename T>
  void logInfo(const char* format, const std::vector<T>& vec);

  template <typename T, std::size_t N>
  void logInfo(const char* format, const std::array<T, N>& arr);

  void logInfo(const char* format, const KDL::JntArray& jnt_array);

  void logInfo(const char* format, const KDL::Vector& vector);

  void logInfo(const char* format, const KDL::Twist& twist);

  void logInfo(const char* format, const KDL::Jacobian& jacobian);

  // logwarning
  template <typename... Args>
  void logWarning(const char* format, Args... args);

  template <typename T, size_t N>
  void logWarning(const std::array<T, N>& arr);

  template <typename T>
  void logWarning(const std::vector<T>& vec);

  template <typename T>
  void logWarning(const std::vector<std::vector<T>>& vec_of_vec);

  void logWarning(const KDL::JntArray& jnt_array);

  template <typename T>
  void format_helper(std::stringstream& ss, const std::string& format, T arg);

  template <typename... Args>
  std::string string_format(const std::string& format, Args... args);

  template <typename T>
  void logWarning(const char* format, const std::vector<T>& vec);

  template <typename T, std::size_t N>
  void logWarning(const char* format, const std::array<T, N>& arr);

  void logWarning(const char* format, const KDL::JntArray& jnt_array);

  // logerror
  template <typename T, size_t N>
  void logError(const std::array<T, N>& arr);

  template <typename T>
  void logError(const std::vector<T>& vec);

  template <typename T>
  void logError(const std::vector<std::vector<T>>& vec_of_vec);

  void logError(const KDL::JntArray& jnt_array);

  template <typename... Args>
  void logError(const char* format, Args... args);

  template <typename T>
  void logError(const char* format, const std::vector<T>& vec);

  template <typename T, std::size_t N>
  void logError(const char* format, const std::array<T, N>& arr);

  void logError(const char* format, const KDL::JntArray& jnt_array);

private:
  bool log_to_terminal_;
  bool log_to_file_;
  bool show_date_;
  std::string logs_dir_;
  std::ofstream log_file_;

  std::string currentTimestamp();

  template <typename... Args>
  void logFormattedMessage(LogLevel level, const std::string& format, Args... args);

  void logMessage(Logger::LogLevel level, const std::string& msg);
};

namespace LoggerUtil
{
template <typename T>
std::string containerToString(const T& container);

template <typename T>
std::string vectorOfVectorToString(const std::vector<std::vector<T>>& vec_of_vec);

std::string jntArrayToString(const KDL::JntArray& jnt_array);
}  // namespace LoggerUtil

#endif  // LOGGER_HPP
