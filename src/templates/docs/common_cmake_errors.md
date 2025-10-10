# Common CMake Errors

This file contians some bugs I've ran into with this CMake build system. Some of these are really hard to catch, so look through this to see if yours is in here. If a new bug is found, put it and the fix for it in here.

## Errors

- Package fails to build when linking libraries (using the `target_link_libraries()` command). This causes a conflict with `ament_target_dependencies()`, but only when declaring the library as `PRIVATE`, `PUBLIC`, or `INTERFACE`.
  - Error: `The keyword signature for target_link_libraries has already been used with the target "<node_name>".  All uses of target_link_libraries with a target must be either all-keyword or all-plain.`
  - Fix: remove the above `PRIVATE`, `PUBLIC`, or `INTERFACE` declaration.

- Package fails to build when adding executables to the package (using the `add_executable()` command).
  - Error: `in function 'function': <item>.c:(.text+0xf28): undefined reference to 'subfunction'`
  - Fix: look through the `#include`s of the `<item>.c` file, then add these to the executable sources. Repeat until the error goes away.
