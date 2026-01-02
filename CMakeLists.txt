cmake_minimum_required(VERSION 3.8)
project(the_cruiser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)

# 2. Include Directories
include_directories(include)

# 3. The Logic Library (Layer 2 & 3)
add_library(${PROJECT_NAME}_lib
  src/implementations/catmull_rom_generator.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# 4. The Node Executable (Layer 4)
add_executable(cruiser_node src/nodes/cruiser_node.cpp)
target_link_libraries(cruiser_node ${PROJECT_NAME}_lib angles::angles)
ament_target_dependencies(cruiser_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  angles
)

# 5. TESTING (INSERTED HERE - BEFORE Install/Package)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Register the Production Test
  ament_add_gtest(test_trajectory_generator tests/test_trajectory_generator.cpp)
  
  # Link against our library
  target_link_libraries(test_trajectory_generator ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_trajectory_generator rclcpp geometry_msgs)
endif()

# 6. Install Rules
install(TARGETS ${PROJECT_NAME}_lib cruiser_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Installs the executable
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 7. Export and Package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

ament_package()
