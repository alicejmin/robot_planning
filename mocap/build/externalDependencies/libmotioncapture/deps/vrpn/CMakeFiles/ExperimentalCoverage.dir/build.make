# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build

# Utility rule file for ExperimentalCoverage.

# Include the progress variables for this target.
include externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/progress.make

externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage:
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture/deps/vrpn && /usr/bin/ctest -D ExperimentalCoverage

ExperimentalCoverage: externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage
ExperimentalCoverage: externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/build.make

.PHONY : ExperimentalCoverage

# Rule to build all files generated by this target.
externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/build: ExperimentalCoverage

.PHONY : externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/build

externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/clean:
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture/deps/vrpn && $(CMAKE_COMMAND) -P CMakeFiles/ExperimentalCoverage.dir/cmake_clean.cmake
.PHONY : externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/clean

externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/depend:
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/deps/vrpn /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture/deps/vrpn /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ExperimentalCoverage.dir/depend

