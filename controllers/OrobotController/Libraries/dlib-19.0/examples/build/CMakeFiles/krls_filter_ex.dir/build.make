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
include CMakeFiles/krls_filter_ex.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/krls_filter_ex.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/krls_filter_ex.dir/flags.make

CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o: CMakeFiles/krls_filter_ex.dir/flags.make
CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o: ../krls_filter_ex.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/thorvat/Optimization_tools/dlib-19.0/examples/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o -c /home/thorvat/Optimization_tools/dlib-19.0/examples/krls_filter_ex.cpp

CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/thorvat/Optimization_tools/dlib-19.0/examples/krls_filter_ex.cpp > CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.i

CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/thorvat/Optimization_tools/dlib-19.0/examples/krls_filter_ex.cpp -o CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.s

CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.requires:
.PHONY : CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.requires

CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.provides: CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.requires
	$(MAKE) -f CMakeFiles/krls_filter_ex.dir/build.make CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.provides.build
.PHONY : CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.provides

CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.provides.build: CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o

# Object files for target krls_filter_ex
krls_filter_ex_OBJECTS = \
"CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o"

# External object files for target krls_filter_ex
krls_filter_ex_EXTERNAL_OBJECTS =

krls_filter_ex: CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o
krls_filter_ex: CMakeFiles/krls_filter_ex.dir/build.make
krls_filter_ex: dlib_build/libdlib.a
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libnsl.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libSM.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libICE.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libX11.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libXext.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libgif.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libpng.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libjpeg.so
krls_filter_ex: /usr/lib/libatlas.so
krls_filter_ex: /usr/lib/libcblas.so
krls_filter_ex: /usr/lib/liblapack.so
krls_filter_ex: /usr/lib/x86_64-linux-gnu/libsqlite3.so
krls_filter_ex: CMakeFiles/krls_filter_ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable krls_filter_ex"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/krls_filter_ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/krls_filter_ex.dir/build: krls_filter_ex
.PHONY : CMakeFiles/krls_filter_ex.dir/build

CMakeFiles/krls_filter_ex.dir/requires: CMakeFiles/krls_filter_ex.dir/krls_filter_ex.cpp.o.requires
.PHONY : CMakeFiles/krls_filter_ex.dir/requires

CMakeFiles/krls_filter_ex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/krls_filter_ex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/krls_filter_ex.dir/clean

CMakeFiles/krls_filter_ex.dir/depend:
	cd /home/thorvat/Optimization_tools/dlib-19.0/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thorvat/Optimization_tools/dlib-19.0/examples /home/thorvat/Optimization_tools/dlib-19.0/examples /home/thorvat/Optimization_tools/dlib-19.0/examples/build /home/thorvat/Optimization_tools/dlib-19.0/examples/build /home/thorvat/Optimization_tools/dlib-19.0/examples/build/CMakeFiles/krls_filter_ex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/krls_filter_ex.dir/depend

