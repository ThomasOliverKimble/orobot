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
CMAKE_SOURCE_DIR = /home/thorvat/Optimization_tools/dlib-19.0/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thorvat/Optimization_tools/dlib-19.0/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/kcentroid_ex.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kcentroid_ex.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kcentroid_ex.dir/flags.make

CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o: CMakeFiles/kcentroid_ex.dir/flags.make
CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o: ../kcentroid_ex.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/thorvat/Optimization_tools/dlib-19.0/examples/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o -c /home/thorvat/Optimization_tools/dlib-19.0/examples/kcentroid_ex.cpp

CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/thorvat/Optimization_tools/dlib-19.0/examples/kcentroid_ex.cpp > CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.i

CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/thorvat/Optimization_tools/dlib-19.0/examples/kcentroid_ex.cpp -o CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.s

CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.requires:
.PHONY : CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.requires

CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.provides: CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.requires
	$(MAKE) -f CMakeFiles/kcentroid_ex.dir/build.make CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.provides.build
.PHONY : CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.provides

CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.provides.build: CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o

# Object files for target kcentroid_ex
kcentroid_ex_OBJECTS = \
"CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o"

# External object files for target kcentroid_ex
kcentroid_ex_EXTERNAL_OBJECTS =

kcentroid_ex: CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o
kcentroid_ex: CMakeFiles/kcentroid_ex.dir/build.make
kcentroid_ex: dlib_build/libdlib.a
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libnsl.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libSM.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libICE.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libX11.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libXext.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libgif.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libpng.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libjpeg.so
kcentroid_ex: /usr/lib/libatlas.so
kcentroid_ex: /usr/lib/libcblas.so
kcentroid_ex: /usr/lib/liblapack.so
kcentroid_ex: /usr/lib/x86_64-linux-gnu/libsqlite3.so
kcentroid_ex: CMakeFiles/kcentroid_ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable kcentroid_ex"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kcentroid_ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kcentroid_ex.dir/build: kcentroid_ex
.PHONY : CMakeFiles/kcentroid_ex.dir/build

CMakeFiles/kcentroid_ex.dir/requires: CMakeFiles/kcentroid_ex.dir/kcentroid_ex.cpp.o.requires
.PHONY : CMakeFiles/kcentroid_ex.dir/requires

CMakeFiles/kcentroid_ex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kcentroid_ex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kcentroid_ex.dir/clean

CMakeFiles/kcentroid_ex.dir/depend:
	cd /home/thorvat/Optimization_tools/dlib-19.0/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thorvat/Optimization_tools/dlib-19.0/examples /home/thorvat/Optimization_tools/dlib-19.0/examples /home/thorvat/Optimization_tools/dlib-19.0/examples/build /home/thorvat/Optimization_tools/dlib-19.0/examples/build /home/thorvat/Optimization_tools/dlib-19.0/examples/build/CMakeFiles/kcentroid_ex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kcentroid_ex.dir/depend

