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
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/build

# Utility rule file for rtabmap_legacy_gencfg.

# Include the progress variables for this target.
include rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/progress.make

rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h
rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/lib/python3/dist-packages/rtabmap_legacy/cfg/CameraConfig.py


/home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h: /home/ubuntu/catkin_ws/src/rtabmap_ros/rtabmap_legacy/cfg/Camera.cfg
/home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/Camera.cfg: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h /home/ubuntu/catkin_ws/devel/lib/python3/dist-packages/rtabmap_legacy/cfg/CameraConfig.py"
	cd /home/ubuntu/catkin_ws/build/rtabmap_ros/rtabmap_legacy && ../../catkin_generated/env_cached.sh /home/ubuntu/catkin_ws/build/rtabmap_ros/rtabmap_legacy/setup_custom_pythonpath.sh /home/ubuntu/catkin_ws/src/rtabmap_ros/rtabmap_legacy/cfg/Camera.cfg /opt/ros/noetic/share/dynamic_reconfigure/cmake/.. /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy /home/ubuntu/catkin_ws/devel/lib/python3/dist-packages/rtabmap_legacy

/home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig.dox: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig.dox

/home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig-usage.dox: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig-usage.dox

/home/ubuntu/catkin_ws/devel/lib/python3/dist-packages/rtabmap_legacy/cfg/CameraConfig.py: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/catkin_ws/devel/lib/python3/dist-packages/rtabmap_legacy/cfg/CameraConfig.py

/home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig.wikidoc: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig.wikidoc

rtabmap_legacy_gencfg: rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg
rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/include/rtabmap_legacy/CameraConfig.h
rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig.dox
rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig-usage.dox
rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/lib/python3/dist-packages/rtabmap_legacy/cfg/CameraConfig.py
rtabmap_legacy_gencfg: /home/ubuntu/catkin_ws/devel/share/rtabmap_legacy/docs/CameraConfig.wikidoc
rtabmap_legacy_gencfg: rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/build.make

.PHONY : rtabmap_legacy_gencfg

# Rule to build all files generated by this target.
rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/build: rtabmap_legacy_gencfg

.PHONY : rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/build

rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/clean:
	cd /home/ubuntu/catkin_ws/build/rtabmap_ros/rtabmap_legacy && $(CMAKE_COMMAND) -P CMakeFiles/rtabmap_legacy_gencfg.dir/cmake_clean.cmake
.PHONY : rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/clean

rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/depend:
	cd /home/ubuntu/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src /home/ubuntu/catkin_ws/src/rtabmap_ros/rtabmap_legacy /home/ubuntu/catkin_ws/build /home/ubuntu/catkin_ws/build/rtabmap_ros/rtabmap_legacy /home/ubuntu/catkin_ws/build/rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rtabmap_ros/rtabmap_legacy/CMakeFiles/rtabmap_legacy_gencfg.dir/depend

