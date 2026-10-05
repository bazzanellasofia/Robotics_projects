cmake_minimum_required(VERSION 3.8)
project(first_project)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(bunker_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)                          # ← AGGIUNGI
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)                # ← AGGIUNGI
find_package(rosidl_default_generators REQUIRED)    # ← AGGIUNGI
find_package(std_msgs REQUIRED)                     # ← AGGIUNGI

# ── Genera il messaggio custom ───────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}          # ← AGGIUNGI
    "msg/TfErrorMsg.msg"                            # ← AGGIUNGI
    DEPENDENCIES std_msgs                           # ← AGGIUNGI                           
)                                                   # ← AGGIUNGI
ament_export_dependencies(rosidl_default_runtime)   # ← AGGIUNGI

# ── Nodo odometer ────────────────────────────────────────────
add_executable(odometer src/odometer.cpp)
ament_target_dependencies(odometer rclcpp nav_msgs bunker_msgs tf2_ros std_srvs)

# ── Nodo tf_error ────────────────────────────────────────────
add_executable(tf_error src/tf_error.cpp)           # ← AGGIUNGI
ament_target_dependencies(tf_error                  # ← AGGIUNGI
    rclcpp                                          # ← AGGIUNGI
    geometry_msgs                                   # ← AGGIUNGI
    tf2                                             # ← AGGIUNGI
    tf2_ros                                         # ← AGGIUNGI
)                                                   # ← AGGIUNGI

# Collega il msg custom a entrambi i nodi (stesso package)
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(odometer ${cpp_typesupport_target})
target_link_libraries(tf_error ${cpp_typesupport_target})   # ← AGGIUNGI

install(TARGETS odometer tf_error                           # ← aggiungi tf_error
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()
ament_package()