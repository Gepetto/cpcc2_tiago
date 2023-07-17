#include "cpcc2_tiago/logger_OCP.hpp"

namespace logger_OCP {
logger::logger(const std::string &filePath,
               const std::map<std::string, int> &columnNames) {
  logger_ = spdlog::basic_logger_mt("logger_OCP", filePath);
  logger_->set_pattern(
      "%v");  // Disable timestamp, thread id and logger name in the output
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
  header_ = oss.str();
  header_.pop_back();  // Delete the last comma
  logger_->info("{}", header_);
}

void logger::log() {  // log what is in data_to_log_
  std::ostringstream oss;
  for (const auto &data : data_to_log_) {
    for (int i = 0; i < data.size(); ++i) {
      oss << data[i] << ",";
    }
  }
  header_ = oss.str();
  header_.pop_back();  // Delete the last comma
  logger_->info("{}", header_);
}

}  // namespace logger_OCP