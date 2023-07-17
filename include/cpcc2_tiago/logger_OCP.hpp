#ifndef LOGGER_OCP_HPP
#define LOGGER_OCP_HPP

#include <Eigen/Dense>
#include <map>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

namespace logger_OCP {
class logger {
public:
  logger(const std::string &filePath,
         const std::map<std::string, int> &columnNames);
  logger(){}; // Default constructor

  template <typename... Args> void log(const Args &...args);

private:
  std::shared_ptr<spdlog::logger> logger_;

  template <typename... Args>
  void appendVectors(std::ostringstream &oss, const Eigen::VectorXd &vector,
                     const Args &...args);

  void writeHeader(const std::map<std::string, int> &columnNames);
};
} // namespace logger_OCP
#endif // LOGGER_OCP_HPP