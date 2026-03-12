#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

#include "file_meta_data.hpp"
#include "log_class/message.hpp"
#include "logger_message_interface/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = std::filesystem;
using std::placeholders::_1;

/**
 * @brief The result of trying to open a file in a directory
 */
enum class FileStatus {
  FILE_OPENED,
  COULD_NOT_OPEN_DIRECTORY,
  NO_DIRECTORY_GIVEN,
};

/**
 * @brief A node that will write all logs it receives into a file
 */
class LogFileWriterNode : public rclcpp::Node {
 public:
  LogFileWriterNode() : Node("LogFileWriterNode") {
    subscription_ = create_subscription<logger_message_interface::msg::Log>(
        "log_topic", 64,
        std::bind(&LogFileWriterNode::topic_callback, this, _1));

    declare_parameter<std::string>("log_file", "");
    std::string log_file_name;
    if (has_parameter("log_file")) {
      get_parameter<std::string>("log_file", log_file_name);
      file_status = open_file(log_file_name);
    } else {
      file_status = FileStatus::NO_DIRECTORY_GIVEN;
    }
  }

  FileStatus get_file_status() const { return file_status; }

 /** 
  *  @brief writes the raw bytes associated  with a type into a file
  *  @tparam T the type that is being written into the file
  *  @param data the data being written
  */ 
  template <typename T>
  void write_to_file(const T* data) {
    log_file.write(reinterpret_cast<const char*>(data), sizeof(T));
  }

 private:

  /**
   * @brief creates a new file within a directory
   * @param directory the directory to search in
   * @return FileStatus::COULD_NOT_OPEN_DIRECTORY if the directory can't be opened
   *         FileStatus::FILE_OPENED if the operation was successful
   */
  FileStatus open_file(const std::string& directory) {
    fs::path dirPath(directory);
    int counter = 1;

    if (!fs::is_directory(dirPath)) {
      file_status = FileStatus::COULD_NOT_OPEN_DIRECTORY;
      std::cerr << "Error: Directory does not exist: " << directory
                << std::endl;
      return FileStatus::COULD_NOT_OPEN_DIRECTORY;
    }

    while (true) {
      std::string newName = "log" + std::to_string(counter) + ".bin";
      fs::path filePath = dirPath / newName;

      if (!fs::exists(filePath)) {
        std::cout << "Log file being written to " << filePath << std::endl;

        log_file = std::ofstream(filePath, std::ios::binary | std::ios::out);

        if (log_file.is_open()) {
          return FileStatus::FILE_OPENED;
        } else {
          return FileStatus::COULD_NOT_OPEN_DIRECTORY;
        }
      }
      counter++;
    }
  }

  void topic_callback(const logger_message_interface::msg::Log& msg) {
    // we are memcpy in the data into a local log becuase reinterpret_casting to
    // a Log* will break C++ strict aliasing and alignment rules
    Log log;
    std::memcpy(&log, msg.data.data(), sizeof(Log));

    // write all constant size stuff into buffer
    write_to_file(&log.time);
    write_to_file(&log.severity);
    write_to_file(&log.source);
    write_to_file(&log.sub_log.type_id);

    // get the size of the non constant stuff
    std::size_t active_size = LogInner::Sizes[log.sub_log.type_id];

    // write all non constant size stuff into buffer
    log_file.write(reinterpret_cast<const char*>(&log.sub_log.storage),
                   active_size);
  }

  rclcpp::Subscription<logger_message_interface::msg::Log>::SharedPtr
      subscription_;
  std::ofstream log_file;
  FileStatus file_status;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LogFileWriterNode>();

  switch (node->get_file_status()) {
    case FileStatus::FILE_OPENED:
      break;
    case FileStatus::COULD_NOT_OPEN_DIRECTORY:
      return 1;
    case FileStatus::NO_DIRECTORY_GIVEN:
      std::cout << "no file was given" << std::endl;
      return 1;
  }

  FileMetaData meta_data;
  node->write_to_file(const_cast<const FileMetaData*>(&meta_data));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}