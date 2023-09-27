cmake_minimum_required(VERSION 3.5)
project(usb2can_fifo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

set(SOURCES
    src/main.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(usb2can_fifo_node ${SOURCES})

target_link_libraries(usb2can_fifo_node 
${CMAKE_CURRENT_SOURCE_DIR}/lib/libusb2can.a 
${CMAKE_CURRENT_SOURCE_DIR}/lib/libftd2xx.a 
-lpthread
)

ament_target_dependencies(usb2can_fifo_node rclcpp std_msgs can_msgs)

install(
  DIRECTORY launch yaml
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()