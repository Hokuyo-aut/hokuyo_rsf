cmake_minimum_required(VERSION 3.8)
project(hokuyo_spel_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)



# hokuyo_spel_master_node

add_executable(hokuyo_spel_master_node
  src/hokuyo_spel_master_node.cpp
  src/payload_converter.cpp
  src/spel_parser.cpp)

target_include_directories(hokuyo_spel_master_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(hokuyo_spel_master_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(hokuyo_spel_master_node
  "rclcpp"
  "rclpy"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "nmea_msgs"
  "diagnostic_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "rosidl_typesupport_cpp"
)

# target_link_libraries(hokuyo_spel_master_node yaml-cpp)

install(TARGETS hokuyo_spel_master_node DESTINATION lib/${PROJECT_NAME})



# hokuyo_spel_ros_node

add_executable(hokuyo_spel_ros_node
  src/hokuyo_spel_ros_node.cpp
  src/payload_converter.cpp
  src/spel_parser.cpp
  src/spel_publisher.cpp)

target_include_directories(hokuyo_spel_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(hokuyo_spel_ros_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(hokuyo_spel_ros_node
  "rclcpp"
  "rclpy"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "nmea_msgs"
  "diagnostic_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "rosidl_typesupport_cpp"
)

# target_link_libraries(hokuyo_spel_ros_node
#   rosidl_typesupport_cpp::rosidl_typesupport_cpp)

install(TARGETS hokuyo_spel_ros_node DESTINATION lib/${PROJECT_NAME})



# send_uint8_command

add_executable(send_uint8_command
  src/send_uint8_command.cpp)

target_include_directories(send_uint8_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(send_uint8_command PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(send_uint8_command
  "rclcpp"
  "std_msgs")

install(TARGETS send_uint8_command
  DESTINATION lib/${PROJECT_NAME})



# send_ip_address

add_executable(send_ip_address
  src/send_ip_address.cpp)

target_include_directories(send_ip_address PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(send_ip_address PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(send_ip_address
  "rclcpp"
  "std_msgs")

install(TARGETS send_ip_address
  DESTINATION lib/${PROJECT_NAME})



# spel_diagnostics_publisher

add_executable(spel_diagnostics_publisher
  src/spel_diagnostics_publisher.cpp)

target_include_directories(spel_diagnostics_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(spel_diagnostics_publisher PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(spel_diagnostics_publisher
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs")

install(TARGETS spel_diagnostics_publisher
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
