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
CMAKE_SOURCE_DIR = /root/CS4501-Labs/project/src/environment_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/CS4501-Labs/project/build/environment_controller

# Utility rule file for environment_controller_gencpp.

# Include the progress variables for this target.
include CMakeFiles/environment_controller_gencpp.dir/progress.make

environment_controller_gencpp: CMakeFiles/environment_controller_gencpp.dir/build.make

.PHONY : environment_controller_gencpp

# Rule to build all files generated by this target.
CMakeFiles/environment_controller_gencpp.dir/build: environment_controller_gencpp

.PHONY : CMakeFiles/environment_controller_gencpp.dir/build

CMakeFiles/environment_controller_gencpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/environment_controller_gencpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/environment_controller_gencpp.dir/clean

CMakeFiles/environment_controller_gencpp.dir/depend:
	cd /root/CS4501-Labs/project/build/environment_controller && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/CS4501-Labs/project/src/environment_controller /root/CS4501-Labs/project/src/environment_controller /root/CS4501-Labs/project/build/environment_controller /root/CS4501-Labs/project/build/environment_controller /root/CS4501-Labs/project/build/environment_controller/CMakeFiles/environment_controller_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/environment_controller_gencpp.dir/depend

