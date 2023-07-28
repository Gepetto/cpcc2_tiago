#ifndef LOGGER_OCP_HPP
#define LOGGER_OCP_HPP

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <map>

namespace logger_OCP {
class logger {
public:
  logger(const std::string &filePath,
         const std::unordered_map<std::string, int> &columnNames);
  logger(){}; // Default constructor

  void log();

  std::vector<Eigen::VectorXd> data_to_log_;

private:
  std::string header_;
  std::shared_ptr<spdlog::logger> logger_;

  void writeHeader(const std::unordered_map<std::string, int> &columnNames);
};
} // namespace logger_OCP
#endif // LOGGER_OCP_HPP
