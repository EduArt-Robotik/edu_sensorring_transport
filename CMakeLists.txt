cmake_minimum_required(VERSION 3.13)

project(edu_sensorring_transport
  LANGUAGES CXX
  VERSION 1.0.0
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library target
add_library(sensorring_transport STATIC
  src/MessageAssembler.cpp
  src/MessageReassembler.cpp
  src/can/CanCodec.cpp
)

add_library(sensorring_transport::sensorring_transport ALIAS sensorring_transport)

target_include_directories(sensorring_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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

  add_executable(sensorring_transport_tests
    tests/test_MessageAssembler.cpp
    tests/test_MessageReassembler.cpp
    tests/test_CanCodec.cpp
  )

  target_link_libraries(sensorring_transport_tests PRIVATE
    sensorring_transport::sensorring_transport
    Catch2::Catch2WithMain
  )

  catch_discover_tests(sensorring_transport_tests
    PROPERTIES LABELS "transport"
  )
endif()
