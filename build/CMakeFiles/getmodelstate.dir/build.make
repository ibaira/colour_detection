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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/baira/catkin_ws/devel/colour_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/baira/catkin_ws/devel/colour_detection/build

# Include any dependencies generated for this target.
include CMakeFiles/getmodelstate.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/getmodelstate.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/getmodelstate.dir/flags.make

CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o: CMakeFiles/getmodelstate.dir/flags.make
CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o: ../src/getmodelstate.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/baira/catkin_ws/devel/colour_detection/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o -c /home/baira/catkin_ws/devel/colour_detection/src/getmodelstate.cpp

CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/baira/catkin_ws/devel/colour_detection/src/getmodelstate.cpp > CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.i

CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/baira/catkin_ws/devel/colour_detection/src/getmodelstate.cpp -o CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.s

CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.requires:
.PHONY : CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.requires

CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.provides: CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.requires
	$(MAKE) -f CMakeFiles/getmodelstate.dir/build.make CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.provides.build
.PHONY : CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.provides

CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.provides.build: CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o

# Object files for target getmodelstate
getmodelstate_OBJECTS = \
"CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o"

# External object files for target getmodelstate
getmodelstate_EXTERNAL_OBJECTS =

getmodelstate: CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o
getmodelstate: /opt/ros/groovy/lib/libroscpp.so
getmodelstate: /usr/lib/x86_64-linux-gnu/libpthread.so
getmodelstate: /usr/lib/libboost_signals-mt.so
getmodelstate: /usr/lib/libboost_filesystem-mt.so
getmodelstate: /usr/lib/libboost_system-mt.so
getmodelstate: /opt/ros/groovy/lib/libcpp_common.so
getmodelstate: /opt/ros/groovy/lib/libroscpp_serialization.so
getmodelstate: /opt/ros/groovy/lib/librostime.so
getmodelstate: /usr/lib/libboost_date_time-mt.so
getmodelstate: /usr/lib/libboost_thread-mt.so
getmodelstate: /opt/ros/groovy/lib/librosconsole.so
getmodelstate: /usr/lib/libboost_regex-mt.so
getmodelstate: /usr/lib/liblog4cxx.so
getmodelstate: /opt/ros/groovy/lib/libxmlrpcpp.so
getmodelstate: /opt/ros/groovy/lib/libimage_transport.so
getmodelstate: /opt/ros/groovy/lib/libmessage_filters.so
getmodelstate: /usr/lib/libtinyxml.so
getmodelstate: /opt/ros/groovy/lib/libclass_loader.so
getmodelstate: /usr/lib/libPocoFoundation.so
getmodelstate: /usr/lib/x86_64-linux-gnu/libdl.so
getmodelstate: /opt/ros/groovy/lib/libconsole_bridge.so
getmodelstate: /opt/ros/groovy/lib/libroslib.so
getmodelstate: /opt/ros/groovy/lib/libcv_bridge.so
getmodelstate: /opt/ros/groovy/lib/libopencv_calib3d.so
getmodelstate: /opt/ros/groovy/lib/libopencv_contrib.so
getmodelstate: /opt/ros/groovy/lib/libopencv_core.so
getmodelstate: /opt/ros/groovy/lib/libopencv_features2d.so
getmodelstate: /opt/ros/groovy/lib/libopencv_flann.so
getmodelstate: /opt/ros/groovy/lib/libopencv_gpu.so
getmodelstate: /opt/ros/groovy/lib/libopencv_highgui.so
getmodelstate: /opt/ros/groovy/lib/libopencv_imgproc.so
getmodelstate: /opt/ros/groovy/lib/libopencv_legacy.so
getmodelstate: /opt/ros/groovy/lib/libopencv_ml.so
getmodelstate: /opt/ros/groovy/lib/libopencv_nonfree.so
getmodelstate: /opt/ros/groovy/lib/libopencv_objdetect.so
getmodelstate: /opt/ros/groovy/lib/libopencv_photo.so
getmodelstate: /opt/ros/groovy/lib/libopencv_stitching.so
getmodelstate: /opt/ros/groovy/lib/libopencv_superres.so
getmodelstate: /opt/ros/groovy/lib/libopencv_ts.so
getmodelstate: /opt/ros/groovy/lib/libopencv_video.so
getmodelstate: /opt/ros/groovy/lib/libopencv_videostab.so
getmodelstate: /opt/ros/groovy/lib/libtf.so
getmodelstate: CMakeFiles/getmodelstate.dir/build.make
getmodelstate: CMakeFiles/getmodelstate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable getmodelstate"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getmodelstate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/getmodelstate.dir/build: getmodelstate
.PHONY : CMakeFiles/getmodelstate.dir/build

CMakeFiles/getmodelstate.dir/requires: CMakeFiles/getmodelstate.dir/src/getmodelstate.cpp.o.requires
.PHONY : CMakeFiles/getmodelstate.dir/requires

CMakeFiles/getmodelstate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/getmodelstate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/getmodelstate.dir/clean

CMakeFiles/getmodelstate.dir/depend:
	cd /home/baira/catkin_ws/devel/colour_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/baira/catkin_ws/devel/colour_detection /home/baira/catkin_ws/devel/colour_detection /home/baira/catkin_ws/devel/colour_detection/build /home/baira/catkin_ws/devel/colour_detection/build /home/baira/catkin_ws/devel/colour_detection/build/CMakeFiles/getmodelstate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/getmodelstate.dir/depend

