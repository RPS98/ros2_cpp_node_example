# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME ros2_cpp_template)
project(${PROJECT_NAME})

# Default to C++17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default build type if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find required dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  std_srvs
  builtin_interfaces
)

# # Comment the following block if not export msgs and srvs
set(SERVICE_DEPENDENCIES
  std_srvs
  std_msgs
  builtin_interfaces
)

set(MSGS_FILES
  "msg/MyTemplateMsg.msg"
)

set(SRVS_FILES
  "srv/MyTemplateSrv.srv"
)

find_package(rosidl_default_generators REQUIRED)

# # End of block
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# # Comment the following block if not export msgs and srvs
# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES}
  DEPENDENCIES ${SERVICE_DEPENDENCIES}
)

# # End of block

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
)

# Set source files
set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
)

# # Comment the following block if not export an executable
# Create the node executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# # End of block

# # Comment the following block if not export a library
# Create the shared library
add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_lib ${PROJECT_DEPENDENCIES})

# # End of block

# # Comment the following block if not export msgs and srvs
if("$ENV{AMENT_CURRENT_RELEASE}" STREQUAL "galactic")
  message(STATUS "ROS 2 Galactic detected")
  rosidl_target_interfaces(${PROJECT_NAME}_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rosidl_target_interfaces(${PROJECT_NAME}_lib
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  message(STATUS "Using Humble as default")
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")
  target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")
endif()

# # End of block

# Set the public include directories for the library
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# # Comment the following block if not export an executable
# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# # End of block

# # Comment the following block if not export a library
# Install the shared library
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the libraries
ament_export_libraries(
  ${PROJECT_NAME}_lib
)

# # End of block

# Export the targets
ament_export_targets(
  export_${PROJECT_NAME}
)

# Export the include directories
ament_export_include_directories(
  include
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# # Comment the following block if not export msgs and srvs
# Export the dependencies of the package
ament_export_dependencies(rosidl_default_runtime)

# # End of block

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)
  # Find additional test dependencies
  set(TEST_DEPENDENCIES
    benchmark
  )

  foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
  endforeach()

  # Find and add test source files
  file(GLOB TEST_SOURCE "tests/*_test.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Add test executable and link necessary dependencies
    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME}_lib)
  endforeach()

  # Include and make available GoogleTest
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)

  # Find and add GTest source files
  file(GLOB TEST_SOURCE "tests/*_gtest.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Add GTest executable and link necessary dependencies
    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest_main ${PROJECT_NAME}_lib)
  endforeach()

  # Find and add benchmark source files
  file(GLOB TEST_SOURCE "tests/*_benchmark.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Add benchmark executable and link necessary dependencies
    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME}_lib)
  endforeach()
endif()

# Package the project using ament
ament_package()
