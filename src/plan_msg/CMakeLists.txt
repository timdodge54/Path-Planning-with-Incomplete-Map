cmake_minimum_required(VERSION 3.8)
project(plan_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# set(msg_files
# "msg/TpssDatum.msg"
# )
set(srv_files
    "srv/Plan.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}

    # ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(geometry_msgs)

# ament_export_dependencies(geographic_msgs)
# ament_export_dependencies(std_msgs)
include_directories(
    ${${PROJECT_NAME}_INCLUDE_DIRS}

    # ${std_msgs_INCLUDE_DIRS}
)

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
