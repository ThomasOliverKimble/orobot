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
include CMakeFiles/multiclass_classification_ex.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/multiclass_classification_ex.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/multiclass_classification_ex.dir/flags.make

CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o: CMakeFiles/multiclass_classification_ex.dir/flags.make
CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o: ../multiclass_classification_ex.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/thorvat/Optimization_tools/dlib-19.0/examples/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o -c /home/thorvat/Optimization_tools/dlib-19.0/examples/multiclass_classification_ex.cpp

CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/thorvat/Optimization_tools/dlib-19.0/examples/multiclass_classification_ex.cpp > CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.i

CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/thorvat/Optimization_tools/dlib-19.0/examples/multiclass_classification_ex.cpp -o CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.s

CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.requires:
.PHONY : CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.requires

CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.provides: CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.requires
	$(MAKE) -f CMakeFiles/multiclass_classification_ex.dir/build.make CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.provides.build
.PHONY : CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.provides

CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.provides.build: CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o

# Object files for target multiclass_classification_ex
multiclass_classification_ex_OBJECTS = \
"CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o"

# External object files for target multiclass_classification_ex
multiclass_classification_ex_EXTERNAL_OBJECTS =

multiclass_classification_ex: CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o
multiclass_classification_ex: CMakeFiles/multiclass_classification_ex.dir/build.make
multiclass_classification_ex: dlib_build/libdlib.a
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libnsl.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libSM.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libICE.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libX11.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libXext.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libgif.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libpng.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libjpeg.so
multiclass_classification_ex: /usr/lib/libatlas.so
multiclass_classification_ex: /usr/lib/libcblas.so
multiclass_classification_ex: /usr/lib/liblapack.so
multiclass_classification_ex: /usr/lib/x86_64-linux-gnu/libsqlite3.so
multiclass_classification_ex: CMakeFiles/multiclass_classification_ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable multiclass_classification_ex"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/multiclass_classification_ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/multiclass_classification_ex.dir/build: multiclass_classification_ex
.PHONY : CMakeFiles/multiclass_classification_ex.dir/build

CMakeFiles/multiclass_classification_ex.dir/requires: CMakeFiles/multiclass_classification_ex.dir/multiclass_classification_ex.cpp.o.requires
.PHONY : CMakeFiles/multiclass_classification_ex.dir/requires

CMakeFiles/multiclass_classification_ex.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/multiclass_classification_ex.dir/cmake_clean.cmake
.PHONY : CMakeFiles/multiclass_classification_ex.dir/clean

CMakeFiles/multiclass_classification_ex.dir/depend:
	cd /home/thorvat/Optimization_tools/dlib-19.0/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thorvat/Optimization_tools/dlib-19.0/examples /home/thorvat/Optimization_tools/dlib-19.0/examples /home/thorvat/Optimization_tools/dlib-19.0/examples/build /home/thorvat/Optimization_tools/dlib-19.0/examples/build /home/thorvat/Optimization_tools/dlib-19.0/examples/build/CMakeFiles/multiclass_classification_ex.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/multiclass_classification_ex.dir/depend

