cmake_minimum_required(VERSION 3.14)
project(ros_motor_uart_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# MQTT(모스키토) 사용 옵션
option(USE_MOSQUITTO "Enable MQTT publish via libmosquitto" ON)

if(USE_MOSQUITTO)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
  add_compile_definitions(USE_MOSQUITTO)
endif()

# ---- core library (ROS 구독 + UART/MQTT 전송 로직) ----
add_library(bridge_core
  src/utils.cpp
  src/mqtt_pub.cpp
  src/uart_port.cpp
  src/motor_bridge_node.cpp
)

# include 폴더
target_include_directories(bridge_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ROS 의존성 (motor_bridge_node.h가 geometry_msgs/std_msgs 포함하므로 필수)
ament_target_dependencies(bridge_core
  rclcpp
  geometry_msgs
  std_msgs
)

# mosquitto 링크
if(USE_MOSQUITTO)
  target_include_directories(bridge_core PUBLIC ${MOSQUITTO_INCLUDE_DIRS})
  target_link_libraries(bridge_core ${MOSQUITTO_LIBRARIES})
endif()

# ---- executable ----
add_executable(motor_bridge_node
  src/main.cpp
)

target_link_libraries(motor_bridge_node bridge_core)

# main.cpp도 헤더 통해 geometry_msgs/std_msgs를 보게 되니까 여기도 deps 명시(안전)
ament_target_dependencies(motor_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
)

# ---- install ----
install(TARGETS motor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
