# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/nvidia/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/catkin_ws/src

# Include any dependencies generated for this target.
include color_tracking/CMakeFiles/color_tracking_node.dir/depend.make

# Include the progress variables for this target.
include color_tracking/CMakeFiles/color_tracking_node.dir/progress.make

# Include the compile flags for this target's objects.
include color_tracking/CMakeFiles/color_tracking_node.dir/flags.make

color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o: color_tracking/CMakeFiles/color_tracking_node.dir/flags.make
color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o: color_tracking/src/color_tracking_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o"
	cd /home/nvidia/catkin_ws/src/color_tracking && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o -c /home/nvidia/catkin_ws/src/color_tracking/src/color_tracking_node.cpp

color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.i"
	cd /home/nvidia/catkin_ws/src/color_tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/catkin_ws/src/color_tracking/src/color_tracking_node.cpp > CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.i

color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.s"
	cd /home/nvidia/catkin_ws/src/color_tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/catkin_ws/src/color_tracking/src/color_tracking_node.cpp -o CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.s

color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.requires:

.PHONY : color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.requires

color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.provides: color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.requires
	$(MAKE) -f color_tracking/CMakeFiles/color_tracking_node.dir/build.make color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.provides.build
.PHONY : color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.provides

color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.provides.build: color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o


# Object files for target color_tracking_node
color_tracking_node_OBJECTS = \
"CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o"

# External object files for target color_tracking_node
color_tracking_node_EXTERNAL_OBJECTS =

devel/lib/color_tracking/color_tracking_node: color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o
devel/lib/color_tracking/color_tracking_node: color_tracking/CMakeFiles/color_tracking_node.dir/build.make
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libcv_bridge.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_core3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libimage_transport.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libclass_loader.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/libPocoFoundation.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libdl.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libroslib.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/librospack.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libpython2.7.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libtinyxml.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_signals.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/librostime.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_system.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libpthread.so
devel/lib/color_tracking/color_tracking_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: /opt/ros/kinetic/lib/libopencv_core3.so.3.3.1
devel/lib/color_tracking/color_tracking_node: color_tracking/CMakeFiles/color_tracking_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../devel/lib/color_tracking/color_tracking_node"
	cd /home/nvidia/catkin_ws/src/color_tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/color_tracking_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
color_tracking/CMakeFiles/color_tracking_node.dir/build: devel/lib/color_tracking/color_tracking_node

.PHONY : color_tracking/CMakeFiles/color_tracking_node.dir/build

color_tracking/CMakeFiles/color_tracking_node.dir/requires: color_tracking/CMakeFiles/color_tracking_node.dir/src/color_tracking_node.cpp.o.requires

.PHONY : color_tracking/CMakeFiles/color_tracking_node.dir/requires

color_tracking/CMakeFiles/color_tracking_node.dir/clean:
	cd /home/nvidia/catkin_ws/src/color_tracking && $(CMAKE_COMMAND) -P CMakeFiles/color_tracking_node.dir/cmake_clean.cmake
.PHONY : color_tracking/CMakeFiles/color_tracking_node.dir/clean

color_tracking/CMakeFiles/color_tracking_node.dir/depend:
	cd /home/nvidia/catkin_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/catkin_ws/src /home/nvidia/catkin_ws/src/color_tracking /home/nvidia/catkin_ws/src /home/nvidia/catkin_ws/src/color_tracking /home/nvidia/catkin_ws/src/color_tracking/CMakeFiles/color_tracking_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : color_tracking/CMakeFiles/color_tracking_node.dir/depend

