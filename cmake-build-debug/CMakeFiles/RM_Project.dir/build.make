# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/pi/Downloads/clion-2024.3.1/bin/cmake/linux/aarch64/bin/cmake

# The command to remove a file.
RM = /home/pi/Downloads/clion-2024.3.1/bin/cmake/linux/aarch64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Downloads/rm_-vision_for_-linux

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/RM_Project.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/RM_Project.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/RM_Project.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RM_Project.dir/flags.make

CMakeFiles/RM_Project.dir/Src/main.cpp.o: CMakeFiles/RM_Project.dir/flags.make
CMakeFiles/RM_Project.dir/Src/main.cpp.o: /home/pi/Downloads/rm_-vision_for_-linux/Src/main.cpp
CMakeFiles/RM_Project.dir/Src/main.cpp.o: CMakeFiles/RM_Project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RM_Project.dir/Src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RM_Project.dir/Src/main.cpp.o -MF CMakeFiles/RM_Project.dir/Src/main.cpp.o.d -o CMakeFiles/RM_Project.dir/Src/main.cpp.o -c /home/pi/Downloads/rm_-vision_for_-linux/Src/main.cpp

CMakeFiles/RM_Project.dir/Src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RM_Project.dir/Src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/rm_-vision_for_-linux/Src/main.cpp > CMakeFiles/RM_Project.dir/Src/main.cpp.i

CMakeFiles/RM_Project.dir/Src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RM_Project.dir/Src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/rm_-vision_for_-linux/Src/main.cpp -o CMakeFiles/RM_Project.dir/Src/main.cpp.s

CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o: CMakeFiles/RM_Project.dir/flags.make
CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o: /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorParam.cpp
CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o: CMakeFiles/RM_Project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o -MF CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o.d -o CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o -c /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorParam.cpp

CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorParam.cpp > CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.i

CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorParam.cpp -o CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.s

CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o: CMakeFiles/RM_Project.dir/flags.make
CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o: /home/pi/Downloads/rm_-vision_for_-linux/Src/armorDescriptor.cpp
CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o: CMakeFiles/RM_Project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o -MF CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o.d -o CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o -c /home/pi/Downloads/rm_-vision_for_-linux/Src/armorDescriptor.cpp

CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/rm_-vision_for_-linux/Src/armorDescriptor.cpp > CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.i

CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/rm_-vision_for_-linux/Src/armorDescriptor.cpp -o CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.s

CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o: CMakeFiles/RM_Project.dir/flags.make
CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o: /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorDetector.cc
CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o: CMakeFiles/RM_Project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o -MF CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o.d -o CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o -c /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorDetector.cc

CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorDetector.cc > CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.i

CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/rm_-vision_for_-linux/Src/ArmorDetector.cc -o CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.s

CMakeFiles/RM_Project.dir/Src/can_usb.cc.o: CMakeFiles/RM_Project.dir/flags.make
CMakeFiles/RM_Project.dir/Src/can_usb.cc.o: /home/pi/Downloads/rm_-vision_for_-linux/Src/can_usb.cc
CMakeFiles/RM_Project.dir/Src/can_usb.cc.o: CMakeFiles/RM_Project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RM_Project.dir/Src/can_usb.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RM_Project.dir/Src/can_usb.cc.o -MF CMakeFiles/RM_Project.dir/Src/can_usb.cc.o.d -o CMakeFiles/RM_Project.dir/Src/can_usb.cc.o -c /home/pi/Downloads/rm_-vision_for_-linux/Src/can_usb.cc

CMakeFiles/RM_Project.dir/Src/can_usb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RM_Project.dir/Src/can_usb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/rm_-vision_for_-linux/Src/can_usb.cc > CMakeFiles/RM_Project.dir/Src/can_usb.cc.i

CMakeFiles/RM_Project.dir/Src/can_usb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RM_Project.dir/Src/can_usb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/rm_-vision_for_-linux/Src/can_usb.cc -o CMakeFiles/RM_Project.dir/Src/can_usb.cc.s

CMakeFiles/RM_Project.dir/Src/controller.cc.o: CMakeFiles/RM_Project.dir/flags.make
CMakeFiles/RM_Project.dir/Src/controller.cc.o: /home/pi/Downloads/rm_-vision_for_-linux/Src/controller.cc
CMakeFiles/RM_Project.dir/Src/controller.cc.o: CMakeFiles/RM_Project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RM_Project.dir/Src/controller.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RM_Project.dir/Src/controller.cc.o -MF CMakeFiles/RM_Project.dir/Src/controller.cc.o.d -o CMakeFiles/RM_Project.dir/Src/controller.cc.o -c /home/pi/Downloads/rm_-vision_for_-linux/Src/controller.cc

CMakeFiles/RM_Project.dir/Src/controller.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RM_Project.dir/Src/controller.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Downloads/rm_-vision_for_-linux/Src/controller.cc > CMakeFiles/RM_Project.dir/Src/controller.cc.i

CMakeFiles/RM_Project.dir/Src/controller.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RM_Project.dir/Src/controller.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Downloads/rm_-vision_for_-linux/Src/controller.cc -o CMakeFiles/RM_Project.dir/Src/controller.cc.s

# Object files for target RM_Project
RM_Project_OBJECTS = \
"CMakeFiles/RM_Project.dir/Src/main.cpp.o" \
"CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o" \
"CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o" \
"CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o" \
"CMakeFiles/RM_Project.dir/Src/can_usb.cc.o" \
"CMakeFiles/RM_Project.dir/Src/controller.cc.o"

# External object files for target RM_Project
RM_Project_EXTERNAL_OBJECTS =

RM_Project: CMakeFiles/RM_Project.dir/Src/main.cpp.o
RM_Project: CMakeFiles/RM_Project.dir/Src/ArmorParam.cpp.o
RM_Project: CMakeFiles/RM_Project.dir/Src/armorDescriptor.cpp.o
RM_Project: CMakeFiles/RM_Project.dir/Src/ArmorDetector.cc.o
RM_Project: CMakeFiles/RM_Project.dir/Src/can_usb.cc.o
RM_Project: CMakeFiles/RM_Project.dir/Src/controller.cc.o
RM_Project: CMakeFiles/RM_Project.dir/build.make
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_alphamat.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_barcode.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_cvv.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_intensity_transform.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_mcc.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_rapid.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_wechat_qrcode.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.6.0
RM_Project: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.6.0
RM_Project: CMakeFiles/RM_Project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable RM_Project"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RM_Project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RM_Project.dir/build: RM_Project
.PHONY : CMakeFiles/RM_Project.dir/build

CMakeFiles/RM_Project.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RM_Project.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RM_Project.dir/clean

CMakeFiles/RM_Project.dir/depend:
	cd /home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Downloads/rm_-vision_for_-linux /home/pi/Downloads/rm_-vision_for_-linux /home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug /home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug /home/pi/Downloads/rm_-vision_for_-linux/cmake-build-debug/CMakeFiles/RM_Project.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/RM_Project.dir/depend

