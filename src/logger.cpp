#include "logger.hpp"

namespace LoggerUtil
{
template <typename T>
std::string containerToString(const T& container)
{
  std::stringstream ss;
  ss << "[";
  for (auto it = container.begin(); it != container.end(); ++it)
  {
    ss << *it;
    if (std::next(it) != container.end())
    {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

template <typename T>
std::string vectorOfVectorToString(const std::vector<std::vector<T>>& vec_of_vec)
{
  std::ostringstream oss;
  oss << "[";
  for (auto it = vec_of_vec.begin(); it != vec_of_vec.end(); ++it)
  {
    if (it != vec_of_vec.begin())
      oss << ", ";
    oss << containerToString(*it);
  }
  oss << "]";
  return oss.str();
}

std::string jntArrayToString(const KDL::JntArray& jnt_array)
{
  std::ostringstream oss;
  oss << "[";
  for (unsigned int i = 0; i < jnt_array.rows(); ++i)
  {
    if (i != 0)
      oss << ", ";
    oss << jnt_array(i);
  }
  oss << "]";
  return oss.str();
}
}  // namespace LoggerUtil

Logger::Logger(bool log_to_terminal, bool log_to_file, std::string logs_dir, bool show_date)
    : log_to_terminal_(log_to_terminal),
      log_to_file_(log_to_file),
      show_date_(show_date),
      logs_dir_(logs_dir)
{
  // Create logs directory if it doesn't exist
  if (!std::filesystem::exists(logs_dir_))
  {
    std::filesystem::create_directories(logs_dir_);
  }

  if (log_to_file_)
  {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    // convert to string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%d_%m_%Y_%H_%M_%S");
    std::string time_str = ss.str();

    // create filename
    std::string file_path = logs_dir_ + time_str + ".log";
    // create file if it doesn't exist
    std::cout << "Creating log file: " << file_path << std::endl;
    std::ofstream file(file_path);
    file.close();

    log_file_.open(file_path, std::ios_base::out | std::ios_base::app);
  }
}

Logger::~Logger()
{
  if (log_to_file_)
  {
    log_file_.close();
  }
}

template <typename T>
void Logger::format_helper(std::stringstream& ss, const std::string& format, T arg)
{
  if constexpr (std::is_same_v<T, int>)
  {
    ss << std::dec << arg;
  }
  else if constexpr (std::is_same_v<T, double>)
  {
    ss << std::fixed << std::setprecision(5) << arg;
  }
  else if constexpr (std::is_same_v<T, std::string>)
  {
    ss << arg;
  }
  else
  {
    throw std::runtime_error("Unsupported argument type.");
  }
}

template <typename... Args>
std::string Logger::string_format(const std::string& format, Args... args)
{
  std::stringstream ss;
  size_t arg_index = 0;
  std::initializer_list<std::variant<int, double, std::string>> arguments = {args...};

  for (size_t i = 0; i < format.size(); ++i)
  {
    if (format[i] == '%')
    {
      if (arg_index >= sizeof...(Args))
      {
        throw std::runtime_error("Not enough arguments for format string.");
      }
      ++i;
      auto it = std::next(arguments.begin(), arg_index);
      std::visit([&](auto&& arg) { format_helper(ss, format.substr(i, 1), arg); },
                 std::variant<int, double, std::string>{*it});
      ++arg_index;
    }
    else
    {
      ss << format[i];
    }
  }

  if (arg_index != sizeof...(Args))
  {
    throw std::runtime_error("Too many arguments for format string.");
  }

  return ss.str();
}

void Logger::test()
{
  logInfo("test");
  logInfo("Vector %d", 7);
  double a = 1.23456789;
  logInfo("Target position: [%f, %f, %f]", a, a, a);
  logWarning("testss");
  logWarning("Vector %d", 7);
  logWarning("Target position: [%f, %f, %f]", a, a, a);
  logError("testss");
  logError("Vector %d", 7);
  logError("Target position: [%f, %f, %f]", a, a, a);

  std::vector<std::string> link_names_vec;
  for (size_t i = 0; i < 3; i++)
  {
    link_names_vec.push_back("as");
  }

  logInfo("Link names: %s", link_names_vec);

  std::array<double, 3> target_position = {0.0, 0.0, 0.0};
  logInfo("Target position: %s", target_position);

  std::array<double, 6> control_accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  logInfo("Control accelerations: %s", control_accelerations);

  KDL::Vector target_position_kdl(0.0, 0.0, 0.0);
  logInfo("Target position kdl: %s", target_position_kdl);

  std::vector<bool> preconditions = {true, false, true};
  logInfo("Preconditions: %s", preconditions);
}

template <typename T, size_t N>
void Logger::logInfo(const std::array<T, N>& arr)
{
  logMessage(INFO, LoggerUtil::containerToString(arr));
}

template <typename T>
void Logger::logInfo(const std::vector<T>& vec)
{
  logMessage(INFO, LoggerUtil::containerToString(vec));
}

template <typename T>
void Logger::logInfo(const std::vector<std::vector<T>>& vec_of_vec)
{
  logMessage(INFO, LoggerUtil::vectorOfVectorToString(vec_of_vec));
}

void Logger::logInfo(const KDL::JntArray& jnt_array)
{
  logMessage(INFO, LoggerUtil::jntArrayToString(jnt_array).c_str());
}

void Logger::logInfo(const KDL::Vector& vec)
{
  // convert to string
  std::ostringstream oss;
  oss << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";

  logMessage(INFO, oss.str().c_str());
}

void Logger::logInfo(const KDL::Twist& twist)
{
  // convert to string
  std::ostringstream oss;
  oss << "[" << twist.vel.x() << ", " << twist.vel.y() << ", " << twist.vel.z() << ", "
      << twist.rot.x() << ", " << twist.rot.y() << ", " << twist.rot.z() << "]";

  logMessage(INFO, oss.str().c_str());
}

void Logger::logInfo(const KDL::Jacobian& jacobian)
{
  // print each row
  for (int i = 0; i < jacobian.columns(); i++)
  {
    // extract the velocity and rotation components of the column
    const auto& vel = jacobian.getColumn(i).vel;
    const auto& rot = jacobian.getColumn(i).rot;

    // combine the components into a single vector
    std::vector<double> col;
    col.insert(col.end(), vel.data, vel.data + 3);
    col.insert(col.end(), rot.data, rot.data + 3);

    // log the vector as a formatted message
    logInfo("Jacobian: %s", LoggerUtil::containerToString(col).c_str());
  }
}

template <typename... Args>
void Logger::logInfo(const char* format, Args... args)
{
  std::string str = string_format(format, args...);
  logFormattedMessage(INFO, str.c_str());
}

template <typename T>
void Logger::logInfo(const char* format, const std::vector<T>& vec)
{
  logFormattedMessage(INFO, format, LoggerUtil::containerToString(vec).c_str());
}

template <typename T, std::size_t N>
void Logger::logInfo(const char* format, const std::array<T, N>& arr)
{
  logFormattedMessage(INFO, format, LoggerUtil::containerToString(arr).c_str());
}

void Logger::logInfo(const char* format, const KDL::JntArray& jnt_array)
{
  logFormattedMessage(INFO, format, LoggerUtil::jntArrayToString(jnt_array).c_str());
}

void Logger::logInfo(const char* format, const KDL::Vector& vec)
{
  // convert to string
  std::ostringstream oss;
  oss << "[" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";

  logFormattedMessage(INFO, format, oss.str().c_str());
}

void Logger::logInfo(const char* format, const KDL::Twist& twist)
{
  // convert to string
  std::ostringstream oss;
  oss << "[" << twist.vel.x() << ", " << twist.vel.y() << ", " << twist.vel.z() << ", "
      << twist.rot.x() << ", " << twist.rot.y() << ", " << twist.rot.z() << "]";

  logFormattedMessage(INFO, format, oss.str().c_str());
}

void Logger::logInfo(const char* format, const KDL::Jacobian& jacobian)
{
  // print each row
  for (int i = 0; i < jacobian.columns(); i++)
  {
    // extract the velocity and rotation components of the column
    const auto& vel = jacobian.getColumn(i).vel;
    const auto& rot = jacobian.getColumn(i).rot;

    // combine the components into a single vector
    std::vector<double> col;
    col.insert(col.end(), vel.data, vel.data + 3);
    col.insert(col.end(), rot.data, rot.data + 3);

    // log the vector as a formatted message
    logFormattedMessage(INFO, format, LoggerUtil::containerToString(col).c_str());
  }
}

// Logger::logWarning
template <typename T, size_t N>
void Logger::logWarning(const std::array<T, N>& arr)
{
  logMessage(WARNING, LoggerUtil::containerToString(arr));
}

template <typename T>
void Logger::logWarning(const std::vector<T>& vec)
{
  logMessage(WARNING, LoggerUtil::containerToString(vec));
}

template <typename T>
void Logger::logWarning(const std::vector<std::vector<T>>& vec_of_vec)
{
  logMessage(WARNING, LoggerUtil::vectorOfVectorToString(vec_of_vec));
}

void Logger::logWarning(const KDL::JntArray& jnt_array)
{
  logMessage(WARNING, LoggerUtil::jntArrayToString(jnt_array));
}

template <typename... Args>
void Logger::logWarning(const char* format, Args... args)
{
  std::string str = string_format(format, args...);
  logFormattedMessage(WARNING, str.c_str());
}

template <typename T>
void Logger::logWarning(const char* format, const std::vector<T>& vec)
{
  logFormattedMessage(WARNING, format, LoggerUtil::containerToString(vec).c_str());
}

template <typename T, std::size_t N>
void Logger::logWarning(const char* format, const std::array<T, N>& arr)
{
  logFormattedMessage(WARNING, format, LoggerUtil::containerToString(arr).c_str());
}

void Logger::logWarning(const char* format, const KDL::JntArray& jnt_array)
{
  logFormattedMessage(WARNING, format, LoggerUtil::jntArrayToString(jnt_array).c_str());
}

// Logger::logError
template <typename T, size_t N>
void Logger::logError(const std::array<T, N>& arr)
{
  logMessage(ERROR, LoggerUtil::containerToString(arr));
}

template <typename T>
void Logger::logError(const std::vector<T>& vec)
{
  logMessage(ERROR, LoggerUtil::containerToString(vec));
}

template <typename T>
void Logger::logError(const std::vector<std::vector<T>>& vec_of_vec)
{
  logMessage(ERROR, LoggerUtil::vectorOfVectorToString(vec_of_vec));
}

void Logger::logError(const KDL::JntArray& jnt_array)
{
  logMessage(ERROR, LoggerUtil::jntArrayToString(jnt_array));
}

template <typename... Args>
void Logger::logError(const char* format, Args... args)
{
  std::string str = string_format(format, args...);
  logFormattedMessage(ERROR, str.c_str());
}

template <typename T>
void Logger::logError(const char* format, const std::vector<T>& vec)
{
  logFormattedMessage(ERROR, format, LoggerUtil::containerToString(vec).c_str());
}

template <typename T, std::size_t N>
void Logger::logError(const char* format, const std::array<T, N>& arr)
{
  logFormattedMessage(ERROR, format, LoggerUtil::containerToString(arr).c_str());
}

void Logger::logError(const char* format, const KDL::JntArray& jnt_array)
{
  logFormattedMessage(ERROR, format, LoggerUtil::jntArrayToString(jnt_array));
}

std::string Logger::currentTimestamp()
{
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_tm = *std::localtime(&now_time_t);
  auto now_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % (60 * 1000);

  std::ostringstream timestamp;
  timestamp << std::put_time(&now_tm, "%H:%M:") << std::setw(5) << std::setfill('0')
            << now_ms.count();
  if (show_date_)
  {
    timestamp << " " << std::put_time(&now_tm, "%d-%m-%Y");
  }
  return "[" + timestamp.str() + "]";
}

template <typename... Args>
void Logger::logFormattedMessage(LogLevel level, const std::string& format, Args... args)
{
  constexpr size_t buf_size = 1024;
  char buffer[buf_size];
  std::snprintf(buffer, buf_size, format.c_str(), args...);
  logMessage(level, buffer);
}

void Logger::logMessage(LogLevel level, const std::string& msg)
{
  std::string level_str;
  std::string level_clr;

  std::string info_color = "\033[0m";
  std::string warning_color = "\033[33m";
  std::string error_color = "\033[31m";
  std::string reset_color = "\033[0m";

  switch (level)
  {
    case INFO:
    {
      level_str = "INFO";
      level_clr = info_color;
      break;
    }
    case WARNING:
    {
      level_str = "WARNING";
      level_clr = warning_color;
      break;
    }
    case ERROR:
    {
      level_str = "ERROR";
      level_clr = error_color;
      break;
    }
    default:
    {
      level_str = "INFO";
      level_clr = info_color;
      break;
    }
  }

  if (log_to_terminal_)
  {
    std::string log_entry =
        reset_color + currentTimestamp() + " " + level_clr + level_str + ": " + msg + reset_color;
    std::cout << log_entry << std::endl;
  }

  if (log_to_file_)
  {
    std::string log_entry = currentTimestamp() + " " + level_str + ": " + msg;
    log_file_ << log_entry << std::endl;
  }
}

// int main() {
//     Logger logger(true, true, false);

//     std::array<int, 3> arr = {1, 2, 3};
//     std::vector<int> vec = {4, 5, 6};
//     std::vector<std::vector<int>> vec_of_vec = {{7, 8}, {9, 10}};
//     KDL::JntArray jnt_array(3);
//     jnt_array(0) = 11;
//     jnt_array(1) = 12;
//     jnt_array(2) = 13;

//     logger.Logger::logInfo("Hello, %s! You have %d messages.", "user", 34);
//     logger.Logger::logInfo(arr);
//     logger.Logger::logWarning(vec);
//     logger.Logger::logInfo(vec_of_vec);
//     logger.Logger::logError(jnt_array);

//     logger.Logger::logInfo("Vector %s with values: %s", "vec", vec);

//     logger.Logger::logInfo("Array %s with values: %s", "name", arr);

//     logger.Logger::logInfo("JntArray %s with values: %s", "name", jnt_array);

//     return 0;
// }