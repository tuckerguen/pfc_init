# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/tucker/research/pfc_init/cmake

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tucker/research/pfc_init/cmake

# Include any dependencies generated for this target.
include CMakeFiles/NeedleMatch.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NeedleMatch.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NeedleMatch.dir/flags.make

CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o: CMakeFiles/NeedleMatch.dir/flags.make
CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o: /home/tucker/research/pfc_init/src/NeedleMatch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o -c /home/tucker/research/pfc_init/src/NeedleMatch.cpp

CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/NeedleMatch.cpp > CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.i

CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/NeedleMatch.cpp -o CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.s

CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.requires:

.PHONY : CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.requires

CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.provides: CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.requires
	$(MAKE) -f CMakeFiles/NeedleMatch.dir/build.make CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.provides.build
.PHONY : CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.provides

CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.provides.build: CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o


# Object files for target NeedleMatch
NeedleMatch_OBJECTS = \
"CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o"

# External object files for target NeedleMatch
NeedleMatch_EXTERNAL_OBJECTS =

NeedleMatch: CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o
NeedleMatch: CMakeFiles/NeedleMatch.dir/build.make
NeedleMatch: CMakeFiles/NeedleMatch.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tucker/research/pfc_init/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable NeedleMatch"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NeedleMatch.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NeedleMatch.dir/build: NeedleMatch

.PHONY : CMakeFiles/NeedleMatch.dir/build

CMakeFiles/NeedleMatch.dir/requires: CMakeFiles/NeedleMatch.dir/home/tucker/research/pfc_init/src/NeedleMatch.cpp.o.requires

.PHONY : CMakeFiles/NeedleMatch.dir/requires

CMakeFiles/NeedleMatch.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NeedleMatch.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NeedleMatch.dir/clean

CMakeFiles/NeedleMatch.dir/depend:
	cd /home/tucker/research/pfc_init/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tucker/research/pfc_init/cmake /home/tucker/research/pfc_init/cmake /home/tucker/research/pfc_init/cmake /home/tucker/research/pfc_init/cmake /home/tucker/research/pfc_init/cmake/CMakeFiles/NeedleMatch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NeedleMatch.dir/depend

