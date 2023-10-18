cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME cpp_template)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})

# Export library headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Add main executable
add_executable(${PROJECT_NAME}_main src/${PROJECT_NAME}_main.cpp)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)

  set(TEST_DEPENDENCIES
    benchmark
  )

  foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
  endforeach()

  file(GLOB TEST_SOURCE "tests/*_test.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()

  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)

  file(GLOB TEST_SOURCE "tests/*_gtest.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest_main ${PROJECT_NAME})
  endforeach()

  file(GLOB TEST_SOURCE "tests/*_benchmark.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
  endforeach()
endif()
