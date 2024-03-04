cmake_minimum_required(VERSION 3.8)
project(mechax_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(leftmechax_trajectory src/lefttrajectoryer.cpp)
ament_target_dependencies(leftmechax_trajectory rclcpp visualization_msgs auto_aim_interfaces geometry_msgs tf2 tf2_ros tf2_geometry_msgs orocos_kdl)
target_link_directories(leftmechax_trajectory PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
target_link_libraries(leftmechax_trajectory ${OpenCV_LIBS} ${orocos_kdl_LIBRARIES})

add_executable(rightmechax_trajectory src/righttrajectoryer.cpp)
ament_target_dependencies(rightmechax_trajectory rclcpp visualization_msgs auto_aim_interfaces geometry_msgs tf2 tf2_ros tf2_geometry_msgs orocos_kdl)
target_link_directories(rightmechax_trajectory PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
target_link_libraries(rightmechax_trajectory ${OpenCV_LIBS} ${orocos_kdl_LIBRARIES})


install(TARGETS
  leftmechax_trajectory
  rightmechax_trajectory
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
