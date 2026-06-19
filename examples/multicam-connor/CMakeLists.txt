# License: Apache 2.0. See LICENSE file in root directory.
cmake_minimum_required(VERSION 3.10)

project(RealsenseExamplesMulticamConnor)

# Headless shared-memory publisher: depends only on the realsense2 library
# (no OpenGL / imgui), so it builds and runs without a display -- e.g. under
# sudo over SSH on macOS.
find_package(Threads REQUIRED)

add_executable(rs-multicam-connor rs-multicam-connor.cpp)
set_property(TARGET rs-multicam-connor PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-multicam-connor ${LRS_TARGET} Threads::Threads)
set_target_properties(rs-multicam-connor PROPERTIES FOLDER Examples)
install(TARGETS rs-multicam-connor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
