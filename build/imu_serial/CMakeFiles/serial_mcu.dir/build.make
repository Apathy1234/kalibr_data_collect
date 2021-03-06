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
CMAKE_SOURCE_DIR = /home/tg/slam_calibration/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tg/slam_calibration/build

# Include any dependencies generated for this target.
include imu_serial/CMakeFiles/serial_mcu.dir/depend.make

# Include the progress variables for this target.
include imu_serial/CMakeFiles/serial_mcu.dir/progress.make

# Include the compile flags for this target's objects.
include imu_serial/CMakeFiles/serial_mcu.dir/flags.make

imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o: imu_serial/CMakeFiles/serial_mcu.dir/flags.make
imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o: /home/tg/slam_calibration/src/imu_serial/src/serial_mcu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tg/slam_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o"
	cd /home/tg/slam_calibration/build/imu_serial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o -c /home/tg/slam_calibration/src/imu_serial/src/serial_mcu.cpp

imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.i"
	cd /home/tg/slam_calibration/build/imu_serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tg/slam_calibration/src/imu_serial/src/serial_mcu.cpp > CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.i

imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.s"
	cd /home/tg/slam_calibration/build/imu_serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tg/slam_calibration/src/imu_serial/src/serial_mcu.cpp -o CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.s

imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.requires:

.PHONY : imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.requires

imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.provides: imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.requires
	$(MAKE) -f imu_serial/CMakeFiles/serial_mcu.dir/build.make imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.provides.build
.PHONY : imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.provides

imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.provides.build: imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o


# Object files for target serial_mcu
serial_mcu_OBJECTS = \
"CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o"

# External object files for target serial_mcu
serial_mcu_EXTERNAL_OBJECTS =

/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: imu_serial/CMakeFiles/serial_mcu.dir/build.make
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/libroscpp.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/librosconsole.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/librostime.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/libcpp_common.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: /opt/ros/kinetic/lib/libserial.so
/home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu: imu_serial/CMakeFiles/serial_mcu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tg/slam_calibration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu"
	cd /home/tg/slam_calibration/build/imu_serial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_mcu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
imu_serial/CMakeFiles/serial_mcu.dir/build: /home/tg/slam_calibration/devel/lib/imu_serial/serial_mcu

.PHONY : imu_serial/CMakeFiles/serial_mcu.dir/build

imu_serial/CMakeFiles/serial_mcu.dir/requires: imu_serial/CMakeFiles/serial_mcu.dir/src/serial_mcu.cpp.o.requires

.PHONY : imu_serial/CMakeFiles/serial_mcu.dir/requires

imu_serial/CMakeFiles/serial_mcu.dir/clean:
	cd /home/tg/slam_calibration/build/imu_serial && $(CMAKE_COMMAND) -P CMakeFiles/serial_mcu.dir/cmake_clean.cmake
.PHONY : imu_serial/CMakeFiles/serial_mcu.dir/clean

imu_serial/CMakeFiles/serial_mcu.dir/depend:
	cd /home/tg/slam_calibration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tg/slam_calibration/src /home/tg/slam_calibration/src/imu_serial /home/tg/slam_calibration/build /home/tg/slam_calibration/build/imu_serial /home/tg/slam_calibration/build/imu_serial/CMakeFiles/serial_mcu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : imu_serial/CMakeFiles/serial_mcu.dir/depend

