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
CMAKE_SOURCE_DIR = /home/yhzhao/temp/LSD-OpenCV-MATLAB

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yhzhao/temp/LSD-OpenCV-MATLAB/build

# Include any dependencies generated for this target.
include CMakeFiles/lsd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lsd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lsd.dir/flags.make

CMakeFiles/lsd.dir/src/lsd.cpp.o: CMakeFiles/lsd.dir/flags.make
CMakeFiles/lsd.dir/src/lsd.cpp.o: ../src/lsd.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yhzhao/temp/LSD-OpenCV-MATLAB/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lsd.dir/src/lsd.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lsd.dir/src/lsd.cpp.o -c /home/yhzhao/temp/LSD-OpenCV-MATLAB/src/lsd.cpp

CMakeFiles/lsd.dir/src/lsd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lsd.dir/src/lsd.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yhzhao/temp/LSD-OpenCV-MATLAB/src/lsd.cpp > CMakeFiles/lsd.dir/src/lsd.cpp.i

CMakeFiles/lsd.dir/src/lsd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lsd.dir/src/lsd.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yhzhao/temp/LSD-OpenCV-MATLAB/src/lsd.cpp -o CMakeFiles/lsd.dir/src/lsd.cpp.s

CMakeFiles/lsd.dir/src/lsd.cpp.o.requires:
.PHONY : CMakeFiles/lsd.dir/src/lsd.cpp.o.requires

CMakeFiles/lsd.dir/src/lsd.cpp.o.provides: CMakeFiles/lsd.dir/src/lsd.cpp.o.requires
	$(MAKE) -f CMakeFiles/lsd.dir/build.make CMakeFiles/lsd.dir/src/lsd.cpp.o.provides.build
.PHONY : CMakeFiles/lsd.dir/src/lsd.cpp.o.provides

CMakeFiles/lsd.dir/src/lsd.cpp.o.provides.build: CMakeFiles/lsd.dir/src/lsd.cpp.o

# Object files for target lsd
lsd_OBJECTS = \
"CMakeFiles/lsd.dir/src/lsd.cpp.o"

# External object files for target lsd
lsd_EXTERNAL_OBJECTS =

lib/liblsd.a: CMakeFiles/lsd.dir/src/lsd.cpp.o
lib/liblsd.a: CMakeFiles/lsd.dir/build.make
lib/liblsd.a: CMakeFiles/lsd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library lib/liblsd.a"
	$(CMAKE_COMMAND) -P CMakeFiles/lsd.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lsd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lsd.dir/build: lib/liblsd.a
.PHONY : CMakeFiles/lsd.dir/build

CMakeFiles/lsd.dir/requires: CMakeFiles/lsd.dir/src/lsd.cpp.o.requires
.PHONY : CMakeFiles/lsd.dir/requires

CMakeFiles/lsd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lsd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lsd.dir/clean

CMakeFiles/lsd.dir/depend:
	cd /home/yhzhao/temp/LSD-OpenCV-MATLAB/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yhzhao/temp/LSD-OpenCV-MATLAB /home/yhzhao/temp/LSD-OpenCV-MATLAB /home/yhzhao/temp/LSD-OpenCV-MATLAB/build /home/yhzhao/temp/LSD-OpenCV-MATLAB/build /home/yhzhao/temp/LSD-OpenCV-MATLAB/build/CMakeFiles/lsd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lsd.dir/depend

