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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mobile/myFusion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mobile/myFusion/build

# Include any dependencies generated for this target.
include src/CMakeFiles/mapcloud.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/mapcloud.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/mapcloud.dir/flags.make

src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o: src/CMakeFiles/mapcloud.dir/flags.make
src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o: ../src/mapcloud.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/mobile/myFusion/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o"
	cd /home/mobile/myFusion/build/src && g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mapcloud.dir/mapcloud.cpp.o -c /home/mobile/myFusion/src/mapcloud.cpp

src/CMakeFiles/mapcloud.dir/mapcloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mapcloud.dir/mapcloud.cpp.i"
	cd /home/mobile/myFusion/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/mobile/myFusion/src/mapcloud.cpp > CMakeFiles/mapcloud.dir/mapcloud.cpp.i

src/CMakeFiles/mapcloud.dir/mapcloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mapcloud.dir/mapcloud.cpp.s"
	cd /home/mobile/myFusion/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/mobile/myFusion/src/mapcloud.cpp -o CMakeFiles/mapcloud.dir/mapcloud.cpp.s

src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.requires:
.PHONY : src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.requires

src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.provides: src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/mapcloud.dir/build.make src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.provides.build
.PHONY : src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.provides

src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.provides.build: src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o

# Object files for target mapcloud
mapcloud_OBJECTS = \
"CMakeFiles/mapcloud.dir/mapcloud.cpp.o"

# External object files for target mapcloud
mapcloud_EXTERNAL_OBJECTS =

../lib/libmapcloud.a: src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o
../lib/libmapcloud.a: src/CMakeFiles/mapcloud.dir/build.make
../lib/libmapcloud.a: src/CMakeFiles/mapcloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../lib/libmapcloud.a"
	cd /home/mobile/myFusion/build/src && $(CMAKE_COMMAND) -P CMakeFiles/mapcloud.dir/cmake_clean_target.cmake
	cd /home/mobile/myFusion/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mapcloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/mapcloud.dir/build: ../lib/libmapcloud.a
.PHONY : src/CMakeFiles/mapcloud.dir/build

src/CMakeFiles/mapcloud.dir/requires: src/CMakeFiles/mapcloud.dir/mapcloud.cpp.o.requires
.PHONY : src/CMakeFiles/mapcloud.dir/requires

src/CMakeFiles/mapcloud.dir/clean:
	cd /home/mobile/myFusion/build/src && $(CMAKE_COMMAND) -P CMakeFiles/mapcloud.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/mapcloud.dir/clean

src/CMakeFiles/mapcloud.dir/depend:
	cd /home/mobile/myFusion/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mobile/myFusion /home/mobile/myFusion/src /home/mobile/myFusion/build /home/mobile/myFusion/build/src /home/mobile/myFusion/build/src/CMakeFiles/mapcloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/mapcloud.dir/depend

