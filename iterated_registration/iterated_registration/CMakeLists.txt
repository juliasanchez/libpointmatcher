cmake_minimum_required(VERSION 2.8.11)

project (iterated_registration)

find_package(libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

add_executable(iterated_registration main.cpp)
target_link_libraries(iterated_registration ${libpointmatcher_LIBRARIES})
