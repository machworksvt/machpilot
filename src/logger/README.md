# Logging Library

This is a set of files to handle the ROS2 logging library.

# Subfiles Overview

### example_logging_node
An example node that demonstrates the logger. It is set up to create a heartbeat log every 0.5 seconds.

### log_class
Contains the C++ log types.

### logger_message_generator
A preprocessor that generates the ROS2 log.

### logger_message_interface
The type that ROS2 uses to send logs from node to node.

### logger_publisher
A publisher that nodes use to produce logs.

### printer_logger_node
A node that reads all logs and prints them to the console.

### log_file_manager
Contains a node that writes logs to a file and an executable that reads those files.

# Using the Logger

Include the project's `log_class` and `logger_publisher` in your project. 

In your node, create an instance of `LoggerPublisher` using the constructor below. It takes a pointer to the parent node and the buffer size (recommended size: 16).

```cpp
LoggerPublisher::LoggerPublisher(rclcpp::Node *node, int buffer_size)
```

Then, call this function to produce a log, where `inner` is the data you want logged and `severity` represents the type of log:

```cpp
void LoggerPublisher::publish(const LogInner &inner, Severity severity)
```

# Adding Your Own Logs
Navigate to the `log_class` project and open `include/log_class/log_types.hpp`.

First, create a class that represents the data you want to log. Even if the data contains only a single member, wrap it in a class. This ensures the code is self-documenting.

Next, write a print function for your new type. Put the implementation for print in `src/log_types.cpp`.

Finally, open `include/log_class/message.hpp` and find:

```cpp
// Defines what types we can log
typedef TriviallyCopyableVariant<...>
    LogInner;
```

Add your type to the end of the list of types. **Warning:** If you insert the type in the middle of the list, you will break serialization/deserialization

# Restrictions on Log Data
1. Trivial Copy/Destruct: Your log type must be `is_trivially_copyable` and `is_trivially_destructible`. This means you cannot have extra logic attached to the move constructor or destructor. If these rules are broken, the code will not compile.

2. No Raw Pointers: Your log cannot contain any raw pointers (even indirectly through types like `std::string_view`). Breaking this rule will not result in a compile error, but will cause runtime issues.

3. Fixed Width Types: I recommend avoiding standard types like `char`, `short`, `int`, `long`, or `long long` because their bit widths are not guaranteed. There is a possibility that this code could run on a system with unexpected bit widths, which would break serialization. Instead, use fixed-width integer types like `int32_t`.

# Serialization / Deserialization

## Serialization (Writing to file)
Run the following command to start a node that dumps logs to a file:

```bash
ros2 run log_file_manager log_file_writer_node --ros-args -p  log_file:="log_files"
```

The `log_files` argument specifies the directory where files will be written. The node will find an unused name in that directory and write the files there.

### Deserialization (Reading from file)

To read the files, run the `log_file_reader`. It is located in `install/log_file_manager/lib/log_file_manager`. Pass the file you want to read as the first argument, and it will print the logs to the console.

**Example:**
```bash
install/log_file_manager/lib/log_file_manager/log_file_reader log_files/log1.bin
```

# Printing Logs to Console
Run the following command to create a node that prints all logs to the console:
```bash
ros2 run printer_logger_node printer_logger_node
```