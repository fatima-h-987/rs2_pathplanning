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
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src/TSP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/src/TSP/build

# Include any dependencies generated for this target.
include CMakeFiles/TSP.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TSP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TSP.dir/flags.make

CMakeFiles/TSP.dir/src/main.cpp.o: CMakeFiles/TSP.dir/flags.make
CMakeFiles/TSP.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/TSP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TSP.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TSP.dir/src/main.cpp.o -c /home/ubuntu/catkin_ws/src/TSP/src/main.cpp

CMakeFiles/TSP.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TSP.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/TSP/src/main.cpp > CMakeFiles/TSP.dir/src/main.cpp.i

CMakeFiles/TSP.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TSP.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/TSP/src/main.cpp -o CMakeFiles/TSP.dir/src/main.cpp.s

CMakeFiles/TSP.dir/src/cities.cpp.o: CMakeFiles/TSP.dir/flags.make
CMakeFiles/TSP.dir/src/cities.cpp.o: ../src/cities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/TSP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/TSP.dir/src/cities.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TSP.dir/src/cities.cpp.o -c /home/ubuntu/catkin_ws/src/TSP/src/cities.cpp

CMakeFiles/TSP.dir/src/cities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TSP.dir/src/cities.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/TSP/src/cities.cpp > CMakeFiles/TSP.dir/src/cities.cpp.i

CMakeFiles/TSP.dir/src/cities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TSP.dir/src/cities.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/TSP/src/cities.cpp -o CMakeFiles/TSP.dir/src/cities.cpp.s

CMakeFiles/TSP.dir/src/solvers.cpp.o: CMakeFiles/TSP.dir/flags.make
CMakeFiles/TSP.dir/src/solvers.cpp.o: ../src/solvers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/TSP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/TSP.dir/src/solvers.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TSP.dir/src/solvers.cpp.o -c /home/ubuntu/catkin_ws/src/TSP/src/solvers.cpp

CMakeFiles/TSP.dir/src/solvers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TSP.dir/src/solvers.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/TSP/src/solvers.cpp > CMakeFiles/TSP.dir/src/solvers.cpp.i

CMakeFiles/TSP.dir/src/solvers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TSP.dir/src/solvers.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/TSP/src/solvers.cpp -o CMakeFiles/TSP.dir/src/solvers.cpp.s

CMakeFiles/TSP.dir/src/utils.cpp.o: CMakeFiles/TSP.dir/flags.make
CMakeFiles/TSP.dir/src/utils.cpp.o: ../src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/TSP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/TSP.dir/src/utils.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TSP.dir/src/utils.cpp.o -c /home/ubuntu/catkin_ws/src/TSP/src/utils.cpp

CMakeFiles/TSP.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TSP.dir/src/utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/TSP/src/utils.cpp > CMakeFiles/TSP.dir/src/utils.cpp.i

CMakeFiles/TSP.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TSP.dir/src/utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/TSP/src/utils.cpp -o CMakeFiles/TSP.dir/src/utils.cpp.s

# Object files for target TSP
TSP_OBJECTS = \
"CMakeFiles/TSP.dir/src/main.cpp.o" \
"CMakeFiles/TSP.dir/src/cities.cpp.o" \
"CMakeFiles/TSP.dir/src/solvers.cpp.o" \
"CMakeFiles/TSP.dir/src/utils.cpp.o"

# External object files for target TSP
TSP_EXTERNAL_OBJECTS =

TSP: CMakeFiles/TSP.dir/src/main.cpp.o
TSP: CMakeFiles/TSP.dir/src/cities.cpp.o
TSP: CMakeFiles/TSP.dir/src/solvers.cpp.o
TSP: CMakeFiles/TSP.dir/src/utils.cpp.o
TSP: CMakeFiles/TSP.dir/build.make
TSP: /usr/lib/x86_64-linux-gnu/libsfml-audio.so
TSP: /usr/lib/x86_64-linux-gnu/libsfml-graphics.so
TSP: /usr/lib/x86_64-linux-gnu/libsfml-network.so
TSP: /usr/lib/x86_64-linux-gnu/libsfml-system.so
TSP: /usr/lib/x86_64-linux-gnu/libsfml-window.so
TSP: CMakeFiles/TSP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/catkin_ws/src/TSP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable TSP"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TSP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TSP.dir/build: TSP

.PHONY : CMakeFiles/TSP.dir/build

CMakeFiles/TSP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TSP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TSP.dir/clean

CMakeFiles/TSP.dir/depend:
	cd /home/ubuntu/catkin_ws/src/TSP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src/TSP /home/ubuntu/catkin_ws/src/TSP /home/ubuntu/catkin_ws/src/TSP/build /home/ubuntu/catkin_ws/src/TSP/build /home/ubuntu/catkin_ws/src/TSP/build/CMakeFiles/TSP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TSP.dir/depend

