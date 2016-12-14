cmake_minimum_required(VERSION 2.8.11)

project (test_filters)

find_package(libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

add_executable(test_filters filter1.cpp)
target_link_libraries(test_filters ${libpointmatcher_LIBRARIES})
