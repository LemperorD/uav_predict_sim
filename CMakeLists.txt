cmake_minimum_required(VERSION 3.16)
project(uav_predict_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(uav_core src/core.cpp)
target_include_directories(uav_core PUBLIC include)
target_link_libraries(uav_core PUBLIC Eigen3::Eigen)
target_compile_options(uav_core PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(core_tests tests/core_tests.cpp)
  target_link_libraries(core_tests PRIVATE uav_core)
  add_test(NAME core_numerics COMMAND core_tests)
endif()
