# Templates

## Node Template

### 1. Standards of Use

There are 6 callbacks native to the lifecycle nodes we are using. Each of these callbacks return a status that the node internally uses to decide which state to change to. See [Lifecycle Nodes](https://www.notion.so/Lifecycle-Nodes-2612748798bd805ab1d2efb34c2fca18) for more details. Including the constructor and destructor, this is 8 functions, all with specific intentions to their usage:

- Constructor: **All Declaration Should Go Here**. Variables should be created here with the intention of being modified later, so make them members of the class. This includes the driver subclass, all measurement variables, internal data, codes for error processing, etc.
- `on_configure()`: this function should load the node's configuration into the previously declared variables, using the launch files hopefully. This step can be moved to the constructor if need be. This should set up the hardware communication preferences, but should not do anything with them yet.
- `on_activate()`: this function should test the functionality of communication, verify if data is being received and is good, and test topic publishing.
- `on_shutdown()`: this function is only called when the node is no longer needed, perform some cleanup, like closing comms and stopping currently running tasks.
- `on_error()`: this function is only called when this node encounters a problem. The previous functions should write some information to the `rclcpp_lifecycle::State` variable in order for this function to handle the error effectively. This will call the destructor on `FAILURE`.
- Destructor: frees all memory associated with this class.

Following these guidelines is a good way to create a readable and standard implementation. All state transitions will be triggered by the `LifecycleManager` node, by publishing information on a topic.

Once this node is in the `ACTIVE` state, it will run its main loop.

## CMakeLists.txt and package.xml Template

If this document doesn't mention parts of this file (`template_CMakeLists.txt`), then those lines are necessary and should not be tampered with.

Please follow all steps listed in the file before removing the lines at the top! Everything after explains when to use the extra stuff in the file.

### Part 1: Dependencies

There are 3 files in which dependencies need to be declared, and all of them have to match for the package to compile:

- in the node/`.cpp` file, with `#include`s. This needs to include every interface file and every package.
- in the `CMakeLists.txt`, with `find_package()` calls. The names for these packages will be the same as the above, but this will only need the interface *package*, not every file. Example: if `#include "std_msgs/msg/float32.hpp"` and `#include "std_msgs/msg/int32.hpp"` are in the `.cpp`, the only line needed in this file is `find_package(std_msgs)`. Do not remove the ones that are already there!!!
- in the `package.xml` file, with `<depend>package</depend>`. This is 1-1 with each `find_package()` call and the names are also the same. Do not remove the ones that are already there!!!

### Part 2: Optional Custom Interfaces

We are building custom interfaces into a single local package. Just follow the steps above with the desired interfaces and it will build.

### Part 3: Libraries

See `--- Library (optional) ---` heading.

To be honest, I'm not sure why we would need this. Don't use this unless discussed with others.

### Part 4: Node Executable

See `--- Node Executable ---`.

`add_executable(<node_name> src/<node_name>.cpp src/<driver_name>.c)` creates an executable object that is installed later to the project. This is the node or nodes. `node_name` is literally the name of the file, `driver_name` is the name of the hardware driver used or any local dependency (that isn't a header file). Multiple files can be put here.

`target_include_directories(<node_name> PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/<node_name>> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include> $<INSTALL_INTERFACE:include>)` lists all
of the locations of the dependencies (that are not packages already). Keep all lines that are already there. It isn't likely one would need to change this.

`ament_target_dependencies(<node_name> rclcpp rclcpp_lifecycle rclcpp_action rosidl_default_generators)` marks all packages as dependencies of the node, so include every name listed in the `find_package()` section.

### Part 5: Installation

See `--- Installation ---`.

`install(TARGETS <node_name> #${PROJECT_NAME}_lib # libraries #  EXPORT export_${PROJECT_NAME} # for child dependencies DESTINATION lib/${PROJECT_NAME} )` installs every previous item to the project, linking it to the node. Uncomment the commented lines if used. Uncomment `EXPORT export_${PROJECT_NAME}` if this package is a dependency for others.

`install(DIRECTORY #launch #config DESTINATION share/${PROJECT_NAME} )` installs the `launch` and `config` folders/files to the ROS2 installation tree. If these are implemented, uncomment the lines to install them.

### Part 6: Tips

Defining Variables:
Sometimes, a path or other value will be reused many times within a CMakeLists file. This can be streamlined by creating environment variables.

- Environment variables are called with the `${VARIABLE}` syntax. All usages will be replaced with the value set to it.
- They can be created and set at the same time with the `set(VARIABLE, "value")` command in CMake. This should be done before the first usage of `${VARIABLE}`.
- If this is setting an environment variable, the name should be unique among the entire build system. Specific naming should be used that refers to the value when creating an environment variable.
