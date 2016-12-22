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
CMAKE_SOURCE_DIR = /home/julia/Desktop/libpointmatcher/metascan_registration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/julia/Desktop/libpointmatcher/metascan_registration/build

# Include any dependencies generated for this target.
include CMakeFiles/metascan_registration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/metascan_registration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/metascan_registration.dir/flags.make

CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o: CMakeFiles/metascan_registration.dir/flags.make
CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o: ../metascan_registration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/julia/Desktop/libpointmatcher/metascan_registration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o -c /home/julia/Desktop/libpointmatcher/metascan_registration/metascan_registration.cpp

CMakeFiles/metascan_registration.dir/metascan_registration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/metascan_registration.dir/metascan_registration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/julia/Desktop/libpointmatcher/metascan_registration/metascan_registration.cpp > CMakeFiles/metascan_registration.dir/metascan_registration.cpp.i

CMakeFiles/metascan_registration.dir/metascan_registration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/metascan_registration.dir/metascan_registration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/julia/Desktop/libpointmatcher/metascan_registration/metascan_registration.cpp -o CMakeFiles/metascan_registration.dir/metascan_registration.cpp.s

CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.requires:

.PHONY : CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.requires

CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.provides: CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.requires
	$(MAKE) -f CMakeFiles/metascan_registration.dir/build.make CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.provides.build
.PHONY : CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.provides

CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.provides.build: CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o


# Object files for target metascan_registration
metascan_registration_OBJECTS = \
"CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o"

# External object files for target metascan_registration
metascan_registration_EXTERNAL_OBJECTS =

metascan_registration: CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o
metascan_registration: CMakeFiles/metascan_registration.dir/build.make
metascan_registration: /usr/local/lib/libpointmatcher.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_thread.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_system.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
metascan_registration: /usr/lib/x86_64-linux-gnu/libpthread.so
metascan_registration: /usr/local/lib/libnabo.a
metascan_registration: CMakeFiles/metascan_registration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/julia/Desktop/libpointmatcher/metascan_registration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable metascan_registration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/metascan_registration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/metascan_registration.dir/build: metascan_registration

.PHONY : CMakeFiles/metascan_registration.dir/build

CMakeFiles/metascan_registration.dir/requires: CMakeFiles/metascan_registration.dir/metascan_registration.cpp.o.requires

.PHONY : CMakeFiles/metascan_registration.dir/requires

CMakeFiles/metascan_registration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/metascan_registration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/metascan_registration.dir/clean

CMakeFiles/metascan_registration.dir/depend:
	cd /home/julia/Desktop/libpointmatcher/metascan_registration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/julia/Desktop/libpointmatcher/metascan_registration /home/julia/Desktop/libpointmatcher/metascan_registration /home/julia/Desktop/libpointmatcher/metascan_registration/build /home/julia/Desktop/libpointmatcher/metascan_registration/build /home/julia/Desktop/libpointmatcher/metascan_registration/build/CMakeFiles/metascan_registration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/metascan_registration.dir/depend
