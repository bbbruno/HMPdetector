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
CMAKE_COMMAND = /home/barbara/Installed_software/CMake/cmake-2.8.12-Linux-i386/bin/cmake

# The command to remove a file.
RM = /home/barbara/Installed_software/CMake/cmake-2.8.12-Linux-i386/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /home/barbara/Installed_software/CMake/cmake-2.8.12-Linux-i386/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/barbara/Documents/HMPdetector

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/barbara/Documents/HMPdetector

# Include any dependencies generated for this target.
include libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/depend.make

# Include the progress variables for this target.
include libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/progress.make

# Include the compile flags for this target's objects.
include libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/flags.make

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/flags.make
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o: libs/GMM+GMR/gmr.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/barbara/Documents/HMPdetector/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/GMM+GMR.dir/gmr.o -c /home/barbara/Documents/HMPdetector/libs/GMM+GMR/gmr.cpp

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GMM+GMR.dir/gmr.i"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/barbara/Documents/HMPdetector/libs/GMM+GMR/gmr.cpp > CMakeFiles/GMM+GMR.dir/gmr.i

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GMM+GMR.dir/gmr.s"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/barbara/Documents/HMPdetector/libs/GMM+GMR/gmr.cpp -o CMakeFiles/GMM+GMR.dir/gmr.s

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.requires:
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.requires

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.provides: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.requires
	$(MAKE) -f libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build.make libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.provides.build
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.provides

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.provides.build: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/flags.make
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o: libs/GMM+GMR/Macros.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/barbara/Documents/HMPdetector/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/GMM+GMR.dir/Macros.o -c /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Macros.cpp

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GMM+GMR.dir/Macros.i"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Macros.cpp > CMakeFiles/GMM+GMR.dir/Macros.i

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GMM+GMR.dir/Macros.s"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Macros.cpp -o CMakeFiles/GMM+GMR.dir/Macros.s

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.requires:
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.requires

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.provides: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.requires
	$(MAKE) -f libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build.make libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.provides.build
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.provides

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.provides.build: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/flags.make
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o: libs/GMM+GMR/Matrix.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/barbara/Documents/HMPdetector/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/GMM+GMR.dir/Matrix.o -c /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Matrix.cpp

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GMM+GMR.dir/Matrix.i"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Matrix.cpp > CMakeFiles/GMM+GMR.dir/Matrix.i

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GMM+GMR.dir/Matrix.s"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Matrix.cpp -o CMakeFiles/GMM+GMR.dir/Matrix.s

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.requires:
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.requires

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.provides: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.requires
	$(MAKE) -f libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build.make libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.provides.build
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.provides

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.provides.build: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/flags.make
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o: libs/GMM+GMR/Vector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/barbara/Documents/HMPdetector/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/GMM+GMR.dir/Vector.o -c /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Vector.cpp

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GMM+GMR.dir/Vector.i"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Vector.cpp > CMakeFiles/GMM+GMR.dir/Vector.i

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GMM+GMR.dir/Vector.s"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/barbara/Documents/HMPdetector/libs/GMM+GMR/Vector.cpp -o CMakeFiles/GMM+GMR.dir/Vector.s

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.requires:
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.requires

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.provides: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.requires
	$(MAKE) -f libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build.make libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.provides.build
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.provides

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.provides.build: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o

# Object files for target GMM+GMR
GMM___GMR_OBJECTS = \
"CMakeFiles/GMM+GMR.dir/gmr.o" \
"CMakeFiles/GMM+GMR.dir/Macros.o" \
"CMakeFiles/GMM+GMR.dir/Matrix.o" \
"CMakeFiles/GMM+GMR.dir/Vector.o"

# External object files for target GMM+GMR
GMM___GMR_EXTERNAL_OBJECTS =

libs/GMM+GMR/libGMM+GMR.a: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o
libs/GMM+GMR/libGMM+GMR.a: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o
libs/GMM+GMR/libGMM+GMR.a: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o
libs/GMM+GMR/libGMM+GMR.a: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o
libs/GMM+GMR/libGMM+GMR.a: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build.make
libs/GMM+GMR/libGMM+GMR.a: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libGMM+GMR.a"
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && $(CMAKE_COMMAND) -P CMakeFiles/GMM+GMR.dir/cmake_clean_target.cmake
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GMM+GMR.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build: libs/GMM+GMR/libGMM+GMR.a
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/build

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/requires: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/gmr.o.requires
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/requires: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Macros.o.requires
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/requires: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Matrix.o.requires
libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/requires: libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/Vector.o.requires
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/requires

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/clean:
	cd /home/barbara/Documents/HMPdetector/libs/GMM+GMR && $(CMAKE_COMMAND) -P CMakeFiles/GMM+GMR.dir/cmake_clean.cmake
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/clean

libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/depend:
	cd /home/barbara/Documents/HMPdetector && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/barbara/Documents/HMPdetector /home/barbara/Documents/HMPdetector/libs/GMM+GMR /home/barbara/Documents/HMPdetector /home/barbara/Documents/HMPdetector/libs/GMM+GMR /home/barbara/Documents/HMPdetector/libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libs/GMM+GMR/CMakeFiles/GMM+GMR.dir/depend

