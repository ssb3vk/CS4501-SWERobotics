# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /root/CS4501-Labs/lab4_ws/src/ros-keyboard

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/CS4501-Labs/lab4_ws/build/keyboard

# Include any dependencies generated for this target.
include CMakeFiles/keyboard.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/keyboard.dir/flags.make

CMakeFiles/keyboard.dir/src/main.cpp.o: CMakeFiles/keyboard.dir/flags.make
CMakeFiles/keyboard.dir/src/main.cpp.o: /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/CS4501-Labs/lab4_ws/build/keyboard/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/keyboard.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/main.cpp.o -c /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/main.cpp

CMakeFiles/keyboard.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/main.cpp > CMakeFiles/keyboard.dir/src/main.cpp.i

CMakeFiles/keyboard.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/main.cpp -o CMakeFiles/keyboard.dir/src/main.cpp.s

CMakeFiles/keyboard.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/keyboard.dir/src/main.cpp.o.requires

CMakeFiles/keyboard.dir/src/main.cpp.o.provides: CMakeFiles/keyboard.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/keyboard.dir/build.make CMakeFiles/keyboard.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/keyboard.dir/src/main.cpp.o.provides

CMakeFiles/keyboard.dir/src/main.cpp.o.provides.build: CMakeFiles/keyboard.dir/src/main.cpp.o


CMakeFiles/keyboard.dir/src/keyboard.cpp.o: CMakeFiles/keyboard.dir/flags.make
CMakeFiles/keyboard.dir/src/keyboard.cpp.o: /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/keyboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/CS4501-Labs/lab4_ws/build/keyboard/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/keyboard.dir/src/keyboard.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/keyboard.cpp.o -c /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/keyboard.cpp

CMakeFiles/keyboard.dir/src/keyboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/keyboard.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/keyboard.cpp > CMakeFiles/keyboard.dir/src/keyboard.cpp.i

CMakeFiles/keyboard.dir/src/keyboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/keyboard.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/CS4501-Labs/lab4_ws/src/ros-keyboard/src/keyboard.cpp -o CMakeFiles/keyboard.dir/src/keyboard.cpp.s

CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires:

.PHONY : CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires

CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides: CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires
	$(MAKE) -f CMakeFiles/keyboard.dir/build.make CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides.build
.PHONY : CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides

CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides.build: CMakeFiles/keyboard.dir/src/keyboard.cpp.o


# Object files for target keyboard
keyboard_OBJECTS = \
"CMakeFiles/keyboard.dir/src/main.cpp.o" \
"CMakeFiles/keyboard.dir/src/keyboard.cpp.o"

# External object files for target keyboard
keyboard_EXTERNAL_OBJECTS =

/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: CMakeFiles/keyboard.dir/src/main.cpp.o
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: CMakeFiles/keyboard.dir/src/keyboard.cpp.o
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: CMakeFiles/keyboard.dir/build.make
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libSDLmain.a
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libSDL.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/libroscpp.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/librosconsole.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/libxmlrpcpp.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/libroscpp_serialization.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/librostime.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /opt/ros/melodic/lib/libcpp_common.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_system.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libpthread.so
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard: CMakeFiles/keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/CS4501-Labs/lab4_ws/build/keyboard/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/keyboard.dir/build: /root/CS4501-Labs/lab4_ws/devel/.private/keyboard/lib/keyboard/keyboard

.PHONY : CMakeFiles/keyboard.dir/build

CMakeFiles/keyboard.dir/requires: CMakeFiles/keyboard.dir/src/main.cpp.o.requires
CMakeFiles/keyboard.dir/requires: CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires

.PHONY : CMakeFiles/keyboard.dir/requires

CMakeFiles/keyboard.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/keyboard.dir/cmake_clean.cmake
.PHONY : CMakeFiles/keyboard.dir/clean

CMakeFiles/keyboard.dir/depend:
	cd /root/CS4501-Labs/lab4_ws/build/keyboard && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/CS4501-Labs/lab4_ws/src/ros-keyboard /root/CS4501-Labs/lab4_ws/src/ros-keyboard /root/CS4501-Labs/lab4_ws/build/keyboard /root/CS4501-Labs/lab4_ws/build/keyboard /root/CS4501-Labs/lab4_ws/build/keyboard/CMakeFiles/keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/keyboard.dir/depend

