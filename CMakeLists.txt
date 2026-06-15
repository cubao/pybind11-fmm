cmake_minimum_required(VERSION 3.15...3.27)

project(
  ${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set(CMAKE_BUILD_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "" FORCE)
  message(STATUS "Set build type to default: ${CMAKE_BUILD_TYPE}")
else()
  message(STATUS "Your build type: ${CMAKE_BUILD_TYPE}")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -ggdb")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

find_package(
  Python REQUIRED
  COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)
find_package(nanobind CONFIG REQUIRED)

file(GLOB SRCS src/*.cpp src/fmm/*.cpp)
nanobind_add_module(_core STABLE_ABI NB_STATIC ${SRCS})
target_include_directories(_core PRIVATE src/3rdparty)

target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# Generate typed stubs (.pyi) and the py.typed marker so editors and static type
# checkers (mypy, pyright, VS Code) understand the compiled module. The
# RECURSIVE flag also emits a stub for the embedded `_core.fmm` submodule.
nanobind_add_stub(
  _core_stub
  MODULE
  _core
  PYTHON_PATH
  $<TARGET_FILE_DIR:_core>
  DEPENDS
  _core
  MARKER_FILE
  py.typed
  RECURSIVE
  OUTPUT_PATH
  ${CMAKE_CURRENT_BINARY_DIR}
  OUTPUT
  _core/__init__.pyi
  _core/fmm.pyi)

install(TARGETS _core DESTINATION pybind11_fmm)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/py.typed DESTINATION pybind11_fmm)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/_core/__init__.pyi
              ${CMAKE_CURRENT_BINARY_DIR}/_core/fmm.pyi
        DESTINATION pybind11_fmm/_core)
