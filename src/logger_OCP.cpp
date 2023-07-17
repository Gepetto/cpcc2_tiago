#include "cpcc2_tiago/logger_OCP.hpp"

namespace logger_OCP {
logger::logger(const std::string &filePath,
               const std::map<std::string, int> &columnNames) {
  logger_ = spdlog::basic_logger_mt("logger_OCP", filePath);
  logger_->set_pattern(
      "%v"); // Disable timestamp, thread id and logger name in the output
  writeHeader(columnNames);
}

void logger::writeHeader(const std::map<std::string, int> &columnNames) {
  std::ostringstream oss;
  for (const auto &pair : columnNames) {
    const std::string &columnName = pair.first;
    const int vec_size = pair.second;
    oss << columnName << ",";
    for (int i = 1; i < vec_size; ++i) {
      oss << ",";
    }
  }
  std::string header = oss.str();
  header.pop_back(); // Delete the last comma
  logger_->info("{}", header);
}

template <typename... Args> void logger::log(const Args &...args) {
  std::ostringstream oss;
  appendVectors(oss, args...);
  logger_->info("{}", oss.str());
}

template <typename... Args>
void logger::appendVectors(std::ostringstream &oss,
                           const Eigen::VectorXd &vector, const Args &...args) {
  oss << vector.transpose() << ", ";
  appendVectors(oss, args...);
}
template <>
void logger::appendVectors(std::ostringstream &oss,
                           const Eigen::VectorXd &vector) {
  oss << vector.transpose();
}
} // namespace logger_OCP