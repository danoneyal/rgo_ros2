cmake_minimum_required(VERSION 3.5)
project(rgo_ros2_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PerceptionEngineSDK REQUIRED)

#added packages 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_package(squid_mc REQUIRED) #RGo messages need to install 
find_package(Eigen3 REQUIRED)


add_executable(rgo_ros2_node 
              src/main.cpp
              src/rgo_ros2_node.cpp
              include/rgo_ros2_node.h)


target_link_libraries(rgo_ros2_node PUBLIC
${PerceptionEngineSDK_LIBRARIES}
)         

#include with no slam
#target_include_directories(rgo_ros2_node PUBLIC
 # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #$<INSTALL_INTERFACE:include>  
#)

target_include_directories(rgo_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PerceptionEngineSDK_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(rgo_ros2_node PUBLIC rclcpp std_msgs geometry_msgs nav_msgs)


install(TARGETS
rgo_ros2_node
  DESTINATION lib/${PROJECT_NAME})

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
