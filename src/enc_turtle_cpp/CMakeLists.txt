cmake_minimum_required(VERSION 3.8)
project(enc_turtle_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 for OpenFHE
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Global settings for TLS support
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DPIC -mcmodel=large -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -DPIC -mcmodel=large -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -Wl,--no-as-needed -ldl -pthread")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pthread")

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncryptedData.msg"
)

# OpenFHE settings
find_package(OpenFHE REQUIRED)

if(OpenFHE_FOUND)
    message(STATUS "Found OpenFHE")
    message(STATUS "OpenFHE Version: ${OpenFHE_VERSION}")
    message(STATUS "OpenFHE Include Dirs: ${OPENFHE_INCLUDE_DIRS}")
    message(STATUS "OpenFHE Libraries: ${OPENFHE_LIBRARIES}")
else()
    message(FATAL_ERROR "OpenFHE not found")
endif()

# Additional include directories for OpenFHE
set(OPENFHE_INCLUDE_DIRS 
    ${OPENFHE_INCLUDE_DIRS}
    /usr/local/include/openfhe
    /usr/local/include/openfhe/core
    /usr/local/include/openfhe/pke
    /usr/local/include/openfhe/binfhe
    /usr/local/include/openfhe/core/lattice
)

include_directories(${OPENFHE_INCLUDE_DIRS})
link_directories(${OPENFHE_LIBDIR})

# Common compile options for OpenFHE executables
set(OPENFHE_COMPILE_OPTIONS
    -fPIC
    -DPIC
    -mcmodel=large
    -pthread
)

set(OPENFHE_LINK_LIBRARIES
    OPENFHEcore
    OPENFHEpke
    OPENFHEbinfhe
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

# Plant node
add_executable(turtle_plant src/turtle_plant.cpp)
target_include_directories(turtle_plant PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  turtle_plant
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

# Controller node
add_executable(turtle_controller src/turtle_controller.cpp)
target_include_directories(turtle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  turtle_controller
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

# Encrypted Plant node
add_executable(enc_turtle_plant src/enc_turtle_plant.cpp)
target_include_directories(enc_turtle_plant PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OPENFHE_INCLUDE_DIRS}
)
target_link_libraries(enc_turtle_plant
  ${OPENFHE_LINK_LIBRARIES}
)
target_compile_options(enc_turtle_plant PRIVATE ${OPENFHE_COMPILE_OPTIONS})
rosidl_target_interfaces(enc_turtle_plant
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(
  enc_turtle_plant
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

# Encrypted Controller node
add_executable(enc_turtle_controller src/enc_turtle_controller.cpp)
target_include_directories(enc_turtle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OPENFHE_INCLUDE_DIRS}
)
target_link_libraries(enc_turtle_controller
  ${OPENFHE_LINK_LIBRARIES}
)
target_compile_options(enc_turtle_controller PRIVATE ${OPENFHE_COMPILE_OPTIONS})
rosidl_target_interfaces(enc_turtle_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(
  enc_turtle_controller
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

# BGV test
add_executable(bgv_test src/bgv_test.cpp)
target_include_directories(bgv_test PUBLIC
  ${OPENFHE_INCLUDE_DIRS}
)
target_link_libraries(bgv_test
  ${OPENFHE_LINK_LIBRARIES}
  OpenMP::OpenMP_CXX
)
target_compile_options(bgv_test PRIVATE -fopenmp ${OPENFHE_COMPILE_OPTIONS})

# Test Plant node
add_executable(test_plant src/test_plant.cpp)
target_include_directories(test_plant PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OPENFHE_INCLUDE_DIRS}
)
target_link_libraries(test_plant
  ${OPENFHE_LINK_LIBRARIES}
)
target_compile_options(test_plant PRIVATE ${OPENFHE_COMPILE_OPTIONS})
rosidl_target_interfaces(test_plant
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(
  test_plant
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

# Test Controller node
add_executable(test_controller src/test_controller.cpp)
target_include_directories(test_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OPENFHE_INCLUDE_DIRS}
)
target_link_libraries(test_controller
  ${OPENFHE_LINK_LIBRARIES}
)
target_compile_options(test_controller PRIVATE ${OPENFHE_COMPILE_OPTIONS})
rosidl_target_interfaces(test_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(
  test_controller
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  enc_turtle_plant
  enc_turtle_controller
  bgv_test
  test_plant
  test_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
