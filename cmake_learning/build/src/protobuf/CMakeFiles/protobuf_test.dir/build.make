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
CMAKE_SOURCE_DIR = /home/llx/learning/cmake_learning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/llx/learning/cmake_learning/build

# Include any dependencies generated for this target.
include src/protobuf/CMakeFiles/protobuf_test.dir/depend.make

# Include the progress variables for this target.
include src/protobuf/CMakeFiles/protobuf_test.dir/progress.make

# Include the compile flags for this target's objects.
include src/protobuf/CMakeFiles/protobuf_test.dir/flags.make

src/protobuf/CMakeFiles/protobuf_test.dir/src/proto_test.cpp.o: src/protobuf/CMakeFiles/protobuf_test.dir/flags.make
src/protobuf/CMakeFiles/protobuf_test.dir/src/proto_test.cpp.o: ../src/protobuf/src/proto_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llx/learning/cmake_learning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/protobuf/CMakeFiles/protobuf_test.dir/src/proto_test.cpp.o"
	cd /home/llx/learning/cmake_learning/build/src/protobuf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/protobuf_test.dir/src/proto_test.cpp.o -c /home/llx/learning/cmake_learning/src/protobuf/src/proto_test.cpp

src/protobuf/CMakeFiles/protobuf_test.dir/src/proto_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/protobuf_test.dir/src/proto_test.cpp.i"
	cd /home/llx/learning/cmake_learning/build/src/protobuf && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llx/learning/cmake_learning/src/protobuf/src/proto_test.cpp > CMakeFiles/protobuf_test.dir/src/proto_test.cpp.i

src/protobuf/CMakeFiles/protobuf_test.dir/src/proto_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/protobuf_test.dir/src/proto_test.cpp.s"
	cd /home/llx/learning/cmake_learning/build/src/protobuf && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llx/learning/cmake_learning/src/protobuf/src/proto_test.cpp -o CMakeFiles/protobuf_test.dir/src/proto_test.cpp.s

# Object files for target protobuf_test
protobuf_test_OBJECTS = \
"CMakeFiles/protobuf_test.dir/src/proto_test.cpp.o"

# External object files for target protobuf_test
protobuf_test_EXTERNAL_OBJECTS =

../src/protobuf/lib/libprotobuf_test.so: src/protobuf/CMakeFiles/protobuf_test.dir/src/proto_test.cpp.o
../src/protobuf/lib/libprotobuf_test.so: src/protobuf/CMakeFiles/protobuf_test.dir/build.make
../src/protobuf/lib/libprotobuf_test.so: src/protobuf/CMakeFiles/protobuf_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/llx/learning/cmake_learning/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../../src/protobuf/lib/libprotobuf_test.so"
	cd /home/llx/learning/cmake_learning/build/src/protobuf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/protobuf_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/protobuf/CMakeFiles/protobuf_test.dir/build: ../src/protobuf/lib/libprotobuf_test.so

.PHONY : src/protobuf/CMakeFiles/protobuf_test.dir/build

src/protobuf/CMakeFiles/protobuf_test.dir/clean:
	cd /home/llx/learning/cmake_learning/build/src/protobuf && $(CMAKE_COMMAND) -P CMakeFiles/protobuf_test.dir/cmake_clean.cmake
.PHONY : src/protobuf/CMakeFiles/protobuf_test.dir/clean

src/protobuf/CMakeFiles/protobuf_test.dir/depend:
	cd /home/llx/learning/cmake_learning/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/llx/learning/cmake_learning /home/llx/learning/cmake_learning/src/protobuf /home/llx/learning/cmake_learning/build /home/llx/learning/cmake_learning/build/src/protobuf /home/llx/learning/cmake_learning/build/src/protobuf/CMakeFiles/protobuf_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/protobuf/CMakeFiles/protobuf_test.dir/depend

