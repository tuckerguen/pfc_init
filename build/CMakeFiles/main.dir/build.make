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
CMAKE_SOURCE_DIR = /home/tucker/research/pfc_init/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tucker/research/pfc_init/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cpp.o: /home/tucker/research/pfc_init/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cpp.o -c /home/tucker/research/pfc_init/src/main.cpp

CMakeFiles/main.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/main.cpp > CMakeFiles/main.dir/main.cpp.i

CMakeFiles/main.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/main.cpp -o CMakeFiles/main.dir/main.cpp.s

CMakeFiles/main.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/main.cpp.o.requires

CMakeFiles/main.dir/main.cpp.o.provides: CMakeFiles/main.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/main.cpp.o.provides

CMakeFiles/main.dir/main.cpp.o.provides.build: CMakeFiles/main.dir/main.cpp.o


CMakeFiles/main.dir/pfc_initializer.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/pfc_initializer.cpp.o: /home/tucker/research/pfc_init/src/pfc_initializer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/pfc_initializer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/pfc_initializer.cpp.o -c /home/tucker/research/pfc_init/src/pfc_initializer.cpp

CMakeFiles/main.dir/pfc_initializer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/pfc_initializer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/pfc_initializer.cpp > CMakeFiles/main.dir/pfc_initializer.cpp.i

CMakeFiles/main.dir/pfc_initializer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/pfc_initializer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/pfc_initializer.cpp -o CMakeFiles/main.dir/pfc_initializer.cpp.s

CMakeFiles/main.dir/pfc_initializer.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/pfc_initializer.cpp.o.requires

CMakeFiles/main.dir/pfc_initializer.cpp.o.provides: CMakeFiles/main.dir/pfc_initializer.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/pfc_initializer.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/pfc_initializer.cpp.o.provides

CMakeFiles/main.dir/pfc_initializer.cpp.o.provides.build: CMakeFiles/main.dir/pfc_initializer.cpp.o


CMakeFiles/main.dir/needle_pose.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/needle_pose.cpp.o: /home/tucker/research/pfc_init/src/needle_pose.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/needle_pose.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/needle_pose.cpp.o -c /home/tucker/research/pfc_init/src/needle_pose.cpp

CMakeFiles/main.dir/needle_pose.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/needle_pose.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/needle_pose.cpp > CMakeFiles/main.dir/needle_pose.cpp.i

CMakeFiles/main.dir/needle_pose.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/needle_pose.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/needle_pose.cpp -o CMakeFiles/main.dir/needle_pose.cpp.s

CMakeFiles/main.dir/needle_pose.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/needle_pose.cpp.o.requires

CMakeFiles/main.dir/needle_pose.cpp.o.provides: CMakeFiles/main.dir/needle_pose.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/needle_pose.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/needle_pose.cpp.o.provides

CMakeFiles/main.dir/needle_pose.cpp.o.provides.build: CMakeFiles/main.dir/needle_pose.cpp.o


CMakeFiles/main.dir/needle_image.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/needle_image.cpp.o: /home/tucker/research/pfc_init/src/needle_image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/needle_image.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/needle_image.cpp.o -c /home/tucker/research/pfc_init/src/needle_image.cpp

CMakeFiles/main.dir/needle_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/needle_image.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/needle_image.cpp > CMakeFiles/main.dir/needle_image.cpp.i

CMakeFiles/main.dir/needle_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/needle_image.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/needle_image.cpp -o CMakeFiles/main.dir/needle_image.cpp.s

CMakeFiles/main.dir/needle_image.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/needle_image.cpp.o.requires

CMakeFiles/main.dir/needle_image.cpp.o.provides: CMakeFiles/main.dir/needle_image.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/needle_image.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/needle_image.cpp.o.provides

CMakeFiles/main.dir/needle_image.cpp.o.provides.build: CMakeFiles/main.dir/needle_image.cpp.o


CMakeFiles/main.dir/needle_template.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/needle_template.cpp.o: /home/tucker/research/pfc_init/src/needle_template.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/needle_template.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/needle_template.cpp.o -c /home/tucker/research/pfc_init/src/needle_template.cpp

CMakeFiles/main.dir/needle_template.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/needle_template.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/needle_template.cpp > CMakeFiles/main.dir/needle_template.cpp.i

CMakeFiles/main.dir/needle_template.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/needle_template.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/needle_template.cpp -o CMakeFiles/main.dir/needle_template.cpp.s

CMakeFiles/main.dir/needle_template.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/needle_template.cpp.o.requires

CMakeFiles/main.dir/needle_template.cpp.o.provides: CMakeFiles/main.dir/needle_template.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/needle_template.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/needle_template.cpp.o.provides

CMakeFiles/main.dir/needle_template.cpp.o.provides.build: CMakeFiles/main.dir/needle_template.cpp.o


CMakeFiles/main.dir/template_match.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/template_match.cpp.o: /home/tucker/research/pfc_init/src/template_match.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/main.dir/template_match.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/template_match.cpp.o -c /home/tucker/research/pfc_init/src/template_match.cpp

CMakeFiles/main.dir/template_match.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/template_match.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/template_match.cpp > CMakeFiles/main.dir/template_match.cpp.i

CMakeFiles/main.dir/template_match.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/template_match.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/template_match.cpp -o CMakeFiles/main.dir/template_match.cpp.s

CMakeFiles/main.dir/template_match.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/template_match.cpp.o.requires

CMakeFiles/main.dir/template_match.cpp.o.provides: CMakeFiles/main.dir/template_match.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/template_match.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/template_match.cpp.o.provides

CMakeFiles/main.dir/template_match.cpp.o.provides.build: CMakeFiles/main.dir/template_match.cpp.o


CMakeFiles/main.dir/pose_helper.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/pose_helper.cpp.o: /home/tucker/research/pfc_init/src/pose_helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/main.dir/pose_helper.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/pose_helper.cpp.o -c /home/tucker/research/pfc_init/src/pose_helper.cpp

CMakeFiles/main.dir/pose_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/pose_helper.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/pose_helper.cpp > CMakeFiles/main.dir/pose_helper.cpp.i

CMakeFiles/main.dir/pose_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/pose_helper.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/pose_helper.cpp -o CMakeFiles/main.dir/pose_helper.cpp.s

CMakeFiles/main.dir/pose_helper.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/pose_helper.cpp.o.requires

CMakeFiles/main.dir/pose_helper.cpp.o.provides: CMakeFiles/main.dir/pose_helper.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/pose_helper.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/pose_helper.cpp.o.provides

CMakeFiles/main.dir/pose_helper.cpp.o.provides.build: CMakeFiles/main.dir/pose_helper.cpp.o


CMakeFiles/main.dir/csv_reader.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/csv_reader.cpp.o: /home/tucker/research/pfc_init/src/csv_reader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/main.dir/csv_reader.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/csv_reader.cpp.o -c /home/tucker/research/pfc_init/src/csv_reader.cpp

CMakeFiles/main.dir/csv_reader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/csv_reader.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/csv_reader.cpp > CMakeFiles/main.dir/csv_reader.cpp.i

CMakeFiles/main.dir/csv_reader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/csv_reader.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/csv_reader.cpp -o CMakeFiles/main.dir/csv_reader.cpp.s

CMakeFiles/main.dir/csv_reader.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/csv_reader.cpp.o.requires

CMakeFiles/main.dir/csv_reader.cpp.o.provides: CMakeFiles/main.dir/csv_reader.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/csv_reader.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/csv_reader.cpp.o.provides

CMakeFiles/main.dir/csv_reader.cpp.o.provides.build: CMakeFiles/main.dir/csv_reader.cpp.o


CMakeFiles/main.dir/matcher.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/matcher.cpp.o: /home/tucker/research/pfc_init/src/matcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/main.dir/matcher.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/matcher.cpp.o -c /home/tucker/research/pfc_init/src/matcher.cpp

CMakeFiles/main.dir/matcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/matcher.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tucker/research/pfc_init/src/matcher.cpp > CMakeFiles/main.dir/matcher.cpp.i

CMakeFiles/main.dir/matcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/matcher.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tucker/research/pfc_init/src/matcher.cpp -o CMakeFiles/main.dir/matcher.cpp.s

CMakeFiles/main.dir/matcher.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/matcher.cpp.o.requires

CMakeFiles/main.dir/matcher.cpp.o.provides: CMakeFiles/main.dir/matcher.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/matcher.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/matcher.cpp.o.provides

CMakeFiles/main.dir/matcher.cpp.o.provides.build: CMakeFiles/main.dir/matcher.cpp.o


# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cpp.o" \
"CMakeFiles/main.dir/pfc_initializer.cpp.o" \
"CMakeFiles/main.dir/needle_pose.cpp.o" \
"CMakeFiles/main.dir/needle_image.cpp.o" \
"CMakeFiles/main.dir/needle_template.cpp.o" \
"CMakeFiles/main.dir/template_match.cpp.o" \
"CMakeFiles/main.dir/pose_helper.cpp.o" \
"CMakeFiles/main.dir/csv_reader.cpp.o" \
"CMakeFiles/main.dir/matcher.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cpp.o
main: CMakeFiles/main.dir/pfc_initializer.cpp.o
main: CMakeFiles/main.dir/needle_pose.cpp.o
main: CMakeFiles/main.dir/needle_image.cpp.o
main: CMakeFiles/main.dir/needle_template.cpp.o
main: CMakeFiles/main.dir/template_match.cpp.o
main: CMakeFiles/main.dir/pose_helper.cpp.o
main: CMakeFiles/main.dir/csv_reader.cpp.o
main: CMakeFiles/main.dir/matcher.cpp.o
main: CMakeFiles/main.dir/build.make
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tucker/research/pfc_init/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/main.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/pfc_initializer.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/needle_pose.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/needle_image.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/needle_template.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/template_match.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/pose_helper.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/csv_reader.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/matcher.cpp.o.requires

.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/tucker/research/pfc_init/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tucker/research/pfc_init/src /home/tucker/research/pfc_init/src /home/tucker/research/pfc_init/build /home/tucker/research/pfc_init/build /home/tucker/research/pfc_init/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

