# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/cmake/1445/bin/cmake

# The command to remove a file.
RM = /snap/cmake/1445/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/noahk/programs/machpilot/src/FSMPilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/noahk/programs/machpilot/src/FSMPilot/build

# Include any dependencies generated for this target.
include CMakeFiles/FSMPilot.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/FSMPilot.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FSMPilot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FSMPilot.dir/flags.make

CMakeFiles/FSMPilot.dir/codegen:
.PHONY : CMakeFiles/FSMPilot.dir/codegen

CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o: CMakeFiles/FSMPilot.dir/flags.make
CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o: /home/noahk/programs/machpilot/src/FSMPilot/FSMPilot.cpp
CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o: CMakeFiles/FSMPilot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/noahk/programs/machpilot/src/FSMPilot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o -MF CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o.d -o CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o -c /home/noahk/programs/machpilot/src/FSMPilot/FSMPilot.cpp

CMakeFiles/FSMPilot.dir/FSMPilot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/FSMPilot.dir/FSMPilot.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/noahk/programs/machpilot/src/FSMPilot/FSMPilot.cpp > CMakeFiles/FSMPilot.dir/FSMPilot.cpp.i

CMakeFiles/FSMPilot.dir/FSMPilot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/FSMPilot.dir/FSMPilot.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/noahk/programs/machpilot/src/FSMPilot/FSMPilot.cpp -o CMakeFiles/FSMPilot.dir/FSMPilot.cpp.s

# Object files for target FSMPilot
FSMPilot_OBJECTS = \
"CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o"

# External object files for target FSMPilot
FSMPilot_EXTERNAL_OBJECTS =

FSMPilot: CMakeFiles/FSMPilot.dir/FSMPilot.cpp.o
FSMPilot: CMakeFiles/FSMPilot.dir/build.make
FSMPilot: CMakeFiles/FSMPilot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/noahk/programs/machpilot/src/FSMPilot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FSMPilot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FSMPilot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FSMPilot.dir/build: FSMPilot
.PHONY : CMakeFiles/FSMPilot.dir/build

CMakeFiles/FSMPilot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FSMPilot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FSMPilot.dir/clean

CMakeFiles/FSMPilot.dir/depend:
	cd /home/noahk/programs/machpilot/src/FSMPilot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/noahk/programs/machpilot/src/FSMPilot /home/noahk/programs/machpilot/src/FSMPilot /home/noahk/programs/machpilot/src/FSMPilot/build /home/noahk/programs/machpilot/src/FSMPilot/build /home/noahk/programs/machpilot/src/FSMPilot/build/CMakeFiles/FSMPilot.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/FSMPilot.dir/depend

