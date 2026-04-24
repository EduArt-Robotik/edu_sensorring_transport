cmake_minimum_required(VERSION 3.13)

project(edu_sensorring_transport
  LANGUAGES CXX
  VERSION 1.0.0
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Strip multi-frame assembly/reassembly support. Intended for the bootloader,
# which speaks a single-frame subset of the protocol and cannot afford the
# std::unordered_map / std::function / std::chrono cost of the reassembler.
option(SENSORRING_TRANSPORT_NO_MULTIFRAME
  "Build without MessageReassembler and without the MessageAssembler multi-frame path (saves flash on constrained targets)"
  OFF
)

# Library target
set(sensorring_transport_sources
  src/MessageAssembler.cpp
  src/can/CanCodec.cpp
)

if(NOT SENSORRING_TRANSPORT_NO_MULTIFRAME)
  list(APPEND sensorring_transport_sources src/MessageReassembler.cpp)
endif()

add_library(sensorring_transport STATIC ${sensorring_transport_sources})

add_library(sensorring_transport::sensorring_transport ALIAS sensorring_transport)

target_include_directories(sensorring_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Propagate the no-multiframe define to consumers so the header guards line up.
if(SENSORRING_TRANSPORT_NO_MULTIFRAME)
  target_compile_definitions(sensorring_transport PUBLIC SENSORRING_TRANSPORT_NO_MULTIFRAME)
endif()

# Tests (only when built standalone or parent enables them)
option(SENSORRING_TRANSPORT_BUILD_TESTS "Build transport module tests" OFF)

# Auto-enable tests when built as top-level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(SENSORRING_TRANSPORT_BUILD_TESTS ON CACHE BOOL "" FORCE)
endif()

if(SENSORRING_TRANSPORT_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
    FIND_PACKAGE_ARGS
  )
  FetchContent_MakeAvailable(Catch2)

  include(CTest)
  include(Catch)

  set(sensorring_transport_test_sources
    tests/test_MessageAssembler.cpp
    tests/test_CanCodec.cpp
  )

  if(NOT SENSORRING_TRANSPORT_NO_MULTIFRAME)
    list(APPEND sensorring_transport_test_sources tests/test_MessageReassembler.cpp)
  endif()

  add_executable(sensorring_transport_tests ${sensorring_transport_test_sources})

  target_link_libraries(sensorring_transport_tests PRIVATE
    sensorring_transport::sensorring_transport
    Catch2::Catch2WithMain
  )

  catch_discover_tests(sensorring_transport_tests
    PROPERTIES LABELS "transport"
  )
endif()
