# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/huo/Documents/ORB_RGBD_SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/huo/Documents/ORB_RGBD_SLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/orb_rgbd_slam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/orb_rgbd_slam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/orb_rgbd_slam.dir/flags.make

CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o: CMakeFiles/orb_rgbd_slam.dir/flags.make
CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o: ../src/main.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/huo/Documents/ORB_RGBD_SLAM/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o -c /home/huo/Documents/ORB_RGBD_SLAM/src/main.cc

CMakeFiles/orb_rgbd_slam.dir/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orb_rgbd_slam.dir/src/main.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/huo/Documents/ORB_RGBD_SLAM/src/main.cc > CMakeFiles/orb_rgbd_slam.dir/src/main.cc.i

CMakeFiles/orb_rgbd_slam.dir/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orb_rgbd_slam.dir/src/main.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/huo/Documents/ORB_RGBD_SLAM/src/main.cc -o CMakeFiles/orb_rgbd_slam.dir/src/main.cc.s

CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.requires:
.PHONY : CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.requires

CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.provides: CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.requires
	$(MAKE) -f CMakeFiles/orb_rgbd_slam.dir/build.make CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.provides.build
.PHONY : CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.provides

CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.provides.build: CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o

# Object files for target orb_rgbd_slam
orb_rgbd_slam_OBJECTS = \
"CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o"

# External object files for target orb_rgbd_slam
orb_rgbd_slam_EXTERNAL_OBJECTS =

../bin/orb_rgbd_slam: CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o
../bin/orb_rgbd_slam: CMakeFiles/orb_rgbd_slam.dir/build.make
../bin/orb_rgbd_slam: ../lib/libORB_RGBD_SLAM.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/orb_rgbd_slam: /home/huo/Downloads/Pangolin-master/build/src/libpangolin.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/orb_rgbd_slam: /usr/lib/x86_64-linux-gnu/libGLEW.so
../bin/orb_rgbd_slam: /usr/local/lib/liboctomap.so
../bin/orb_rgbd_slam: /usr/local/lib/liboctomath.so
../bin/orb_rgbd_slam: CMakeFiles/orb_rgbd_slam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/orb_rgbd_slam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/orb_rgbd_slam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/orb_rgbd_slam.dir/build: ../bin/orb_rgbd_slam
.PHONY : CMakeFiles/orb_rgbd_slam.dir/build

CMakeFiles/orb_rgbd_slam.dir/requires: CMakeFiles/orb_rgbd_slam.dir/src/main.cc.o.requires
.PHONY : CMakeFiles/orb_rgbd_slam.dir/requires

CMakeFiles/orb_rgbd_slam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/orb_rgbd_slam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/orb_rgbd_slam.dir/clean

CMakeFiles/orb_rgbd_slam.dir/depend:
	cd /home/huo/Documents/ORB_RGBD_SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huo/Documents/ORB_RGBD_SLAM /home/huo/Documents/ORB_RGBD_SLAM /home/huo/Documents/ORB_RGBD_SLAM/build /home/huo/Documents/ORB_RGBD_SLAM/build /home/huo/Documents/ORB_RGBD_SLAM/build/CMakeFiles/orb_rgbd_slam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/orb_rgbd_slam.dir/depend
