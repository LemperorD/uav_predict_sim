cmake_minimum_required(VERSION 3.16)
project(uav_predict_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(uav_core src/core.cpp src/experiment.cpp)
target_include_directories(uav_core PUBLIC include)
target_link_libraries(uav_core PUBLIC Eigen3::Eigen)
target_compile_options(uav_core PRIVATE -Wall -Wextra -Wpedantic)

# Reconfigure after a commit to refresh the recorded source revision.
execute_process(COMMAND git describe --always --dirty
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE UAV_GIT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT UAV_GIT_COMMIT)
  set(UAV_GIT_COMMIT unknown)
endif()
target_compile_definitions(uav_core PRIVATE UAV_GIT_COMMIT="${UAV_GIT_COMMIT}")

add_executable(uav_sim apps/uav_sim.cpp)
target_link_libraries(uav_sim PRIVATE uav_core)

option(UAV_ENABLE_PLOTTING "Enable matplotlib-cpp PNG output" OFF)
option(UAV_ENABLE_VIEWER "Enable Pangolin playback" OFF)
if(UAV_ENABLE_PLOTTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
  target_sources(uav_sim PRIVATE src/plotting.cpp)
  target_include_directories(uav_sim SYSTEM PRIVATE third_party/matplotlib-cpp)
  target_link_libraries(uav_sim PRIVATE Python3::Python Python3::NumPy)
  target_compile_definitions(uav_sim PRIVATE UAV_ENABLE_PLOTTING)
endif()
if(UAV_ENABLE_VIEWER)
  find_package(Pangolin CONFIG REQUIRED)
  target_sources(uav_sim PRIVATE src/visualization.cpp)
  target_link_libraries(uav_sim PRIVATE pango_display pango_opengl pango_vars)
  target_compile_definitions(uav_sim PRIVATE UAV_ENABLE_VIEWER)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(core_tests tests/core_tests.cpp)
  target_link_libraries(core_tests PRIVATE uav_core)
  add_test(NAME core_numerics COMMAND core_tests)
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    set(UAV_TEST_PLOT_OPTION "")
    if(UAV_ENABLE_PLOTTING)
      set(UAV_TEST_PLOT_OPTION --plot)
    endif()
    add_test(NAME experiment_outputs
      COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/check_experiment.py
              $<TARGET_FILE:uav_sim> ${UAV_TEST_PLOT_OPTION})
  endif()
endif()
