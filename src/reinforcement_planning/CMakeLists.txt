cmake_minimum_required(VERSION 3.5)
project(reinforcement_planning)

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
find_package(ament_cmake_python REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(map_loader_cmd src/map_loader.cpp)
ament_target_dependencies(map_loader_cmd
  rclcpp
  nav2_msgs
  nav2_msgs
  nav_msgs
  nav2_map_server
)
install(TARGETS
  map_loader_cmd
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_loader.py
  scripts/spawn_robots.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  ddpg_planning
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(PROGRAMS
  scripts/rein_interface.py
  scripts/plan_saver.py
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
