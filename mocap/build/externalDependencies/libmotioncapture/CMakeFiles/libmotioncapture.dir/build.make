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

# Include any dependencies generated for this target.
include externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/depend.make

# Include the progress variables for this target.
include externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/progress.make

# Include the compile flags for this target's objects.
include externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.o: ../externalDependencies/libmotioncapture/src/motioncapture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/motioncapture.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/motioncapture.cpp > CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/motioncapture.cpp -o CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.s

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.o: ../externalDependencies/libmotioncapture/src/testmocap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/testmocap.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/testmocap.cpp > CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/testmocap.cpp -o CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.s

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vicon.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vicon.cpp.o: ../externalDependencies/libmotioncapture/src/vicon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vicon.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/vicon.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/vicon.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vicon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/vicon.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/vicon.cpp > CMakeFiles/libmotioncapture.dir/src/vicon.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vicon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/vicon.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/vicon.cpp -o CMakeFiles/libmotioncapture.dir/src/vicon.cpp.s

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.o: ../externalDependencies/libmotioncapture/src/optitrack.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/optitrack.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/optitrack.cpp > CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/optitrack.cpp -o CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.s

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.o: ../externalDependencies/libmotioncapture/src/optitrack_closed_source.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/optitrack_closed_source.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/optitrack_closed_source.cpp > CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/optitrack_closed_source.cpp -o CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.s

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.o: ../externalDependencies/libmotioncapture/src/qualisys.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/qualisys.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/qualisys.cpp > CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/qualisys.cpp -o CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.s

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.o: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/flags.make
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.o: ../externalDependencies/libmotioncapture/src/vrpn.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.o"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.o -c /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/vrpn.cpp

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.i"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/vrpn.cpp > CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.i

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.s"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture/src/vrpn.cpp -o CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.s

# Object files for target libmotioncapture
libmotioncapture_OBJECTS = \
"CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.o" \
"CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.o" \
"CMakeFiles/libmotioncapture.dir/src/vicon.cpp.o" \
"CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.o" \
"CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.o" \
"CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.o" \
"CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.o"

# External object files for target libmotioncapture
libmotioncapture_EXTERNAL_OBJECTS =

externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/motioncapture.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/testmocap.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vicon.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/optitrack_closed_source.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/qualisys.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/src/vrpn.cpp.o
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/build.make
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/deps/vicon-datastream-sdk/libViconDataStreamSDK_CPP.a
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/deps/qualisys_cpp_sdk/libqualisys_cpp_sdk.so
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/deps/vrpn/libvrpn.so.0
externalDependencies/libmotioncapture/liblibmotioncapture.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
externalDependencies/libmotioncapture/liblibmotioncapture.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
externalDependencies/libmotioncapture/liblibmotioncapture.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/deps/vrpn/quat/libquat.so
externalDependencies/libmotioncapture/liblibmotioncapture.so: externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library liblibmotioncapture.so"
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libmotioncapture.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/build: externalDependencies/libmotioncapture/liblibmotioncapture.so

.PHONY : externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/build

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/clean:
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture && $(CMAKE_COMMAND) -P CMakeFiles/libmotioncapture.dir/cmake_clean.cmake
.PHONY : externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/clean

externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/depend:
	cd /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/externalDependencies/libmotioncapture /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture /home/alice/crazyswarm/ros_ws/src/crazyswarm/scripts/utra/mocap/build/externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : externalDependencies/libmotioncapture/CMakeFiles/libmotioncapture.dir/depend

