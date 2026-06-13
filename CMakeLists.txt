cmake_minimum_required(VERSION 3.21)

project(hanfeng LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS "Run only plane-surface real model weld tests" OFF)
option(HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS "Run only surface-surface real model weld tests" OFF)

set(HANFENG_HEADERS
  include/hanfeng/geometry.hpp
  include/hanfeng/panel.hpp
  include/hanfeng/weld.hpp
  include/hanfeng/plane_surface_hanfeng_util.hpp
  include/hanfeng/hanfeng.hpp
)

set(HANFENG_TEST_HEADERS
  tests/test_common.hpp
)

set(HANFENG_SOURCES
  src/geometry.cpp
  src/panel.cpp
  src/plane_surface_hanfeng_util.cpp
  src/hanfeng.cpp
)

set(HANFENG_TEST_SOURCES
  tests/base_geometry_test.cpp
  tests/data_structures_layout_test.cpp
  tests/model_io_test.cpp
  tests/plane_surface_weld_test.cpp
  tests/test_main.cpp
)

add_executable(hanfeng
  ${HANFENG_HEADERS}
  ${HANFENG_TEST_HEADERS}
  ${HANFENG_SOURCES}
  ${HANFENG_TEST_SOURCES}
)

target_include_directories(hanfeng
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

target_compile_definitions(hanfeng
  PRIVATE
    HANFENG_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=$<BOOL:${HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS}>
    HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=$<BOOL:${HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS}>
)

if (MSVC)
  target_compile_options(hanfeng PRIVATE /utf-8)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "Header Files"
  FILES ${HANFENG_HEADERS}
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/tests" PREFIX "Header Files\\tests"
  FILES ${HANFENG_TEST_HEADERS}
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files"
  FILES ${HANFENG_SOURCES}
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/tests" PREFIX "Source Files\\tests"
  FILES ${HANFENG_TEST_SOURCES}
)
