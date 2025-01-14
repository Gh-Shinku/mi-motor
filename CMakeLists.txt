cmake_minimum_required(VERSION 3.15)
project(mi_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") # required by clangd lsp
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bupt_can REQUIRED)

add_executable(motor_node src/motor_node.cpp)
target_include_directories(motor_node PRIVATE include)
target_link_libraries(motor_node PUBLIC bupt_can)

ament_target_dependencies(motor_node PUBLIC rclcpp std_msgs)

add_executable(send_node test/send_node.cpp)
target_include_directories(send_node PRIVATE include)

ament_target_dependencies(send_node PUBLIC rclcpp std_msgs)

ament_package()

install(TARGETS motor_node send_node DESTINATION lib/${PROJECT_NAME})
