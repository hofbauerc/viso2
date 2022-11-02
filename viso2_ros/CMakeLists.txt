cmake_minimum_required(VERSION 3.5)
project(viso2_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(libviso2 REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(msg_files
  "msg/VisoInfo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)


#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/VisoInfo.msg"
#  DEPENDENCIES builtin_interfaces std_msgs
#  ADD_LINTER_TESTS
#)



include_directories(
    include
    src
    ${libviso2_INCLUDE_DIRS}/libviso2
    ${catkin_INCLUDE_DIRS}
)

add_executable(stereo_odometer src/stereo_odometer.cpp)

add_executable(transform_odom_node
  src/transform_odom_node.cpp
  src/transform_odom.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stereo_odometer "${cpp_typesupport_target}") 

ament_target_dependencies(stereo_odometer
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs"
  "std_srvs"
  "diagnostic_msgs"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
  "image_transport"
  "libviso2"
  "image_geometry"
  "cv_bridge"
  "OpenCV"
)

ament_target_dependencies(transform_odom_node 
  rclcpp 
  Eigen3
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
        stereo_odometer
        transform_odom_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

