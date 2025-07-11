cmake_minimum_required(VERSION 3.8)
project(multi_drone_slung_load_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(image_transport REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(multi_drone_slung_load_interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
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

# Declare libraries and executables
add_library(State src/State.cpp)
add_library(utils src/utils.cpp)
add_library(frame_transforms src/frame_transforms.cpp)

add_executable(pixhawk src/pixhawk.cpp)
add_executable(gcs_background_centralized src/gcs_background_centralized.cpp)
add_executable(tf_static_to_tf_relay src/tf_static_to_tf_relay.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(gcs_background_centralized
  #State
  utils
  #frame_transforms
)

target_link_libraries(pixhawk
  State
  utils
  frame_transforms
)

target_link_libraries(utils
  State
)

target_link_libraries(State
  utils
)

# target_link_libraries(logger
#   State
#   utils
#   #multi_drone_slung_load_interfaces
# )

ament_target_dependencies(State rclcpp tf2_geometry_msgs tf2 Eigen3)
ament_target_dependencies(utils rclcpp tf2_ros tf2_eigen Eigen3 std_msgs geometry_msgs tf2_geometry_msgs px4_msgs multi_drone_slung_load_interfaces) #OpenCV
ament_target_dependencies(frame_transforms Eigen3 tf2_geometry_msgs sensor_msgs)
ament_target_dependencies(pixhawk rclcpp geometry_msgs px4_msgs multi_drone_slung_load_interfaces) # Eigen3 std_msgs sensor_msgs cv_bridge image_transport OpenCV tf2_geometry_msgs multi_drone_slung_load_interfaces)
ament_target_dependencies(gcs_background_centralized rclcpp geometry_msgs px4_msgs multi_drone_slung_load_interfaces) #std_msgs tf2_geometry_msgs
ament_target_dependencies(tf_static_to_tf_relay rclcpp tf2_msgs geometry_msgs)

# Install the libraries and executables
#install(TARGETS
#  utils
#  State
#  pixhawk
#  gcs_background_centralized
#  tf_static_to_tf_relay
#  EXPORT multi_drone_slung_load_cppTargets
#  DESTINATION lib/${PROJECT_NAME})

# Install and export only shared libraries
install(TARGETS
  utils
  State
  EXPORT multi_drone_slung_load_cppTargets
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables (DO NOT EXPORT)
install(TARGETS
  frame_transforms
  pixhawk
  gcs_background_centralized
  tf_static_to_tf_relay
  DESTINATION lib/${PROJECT_NAME}
)



# Install the header files for inclusion in other packages
install(DIRECTORY include/
  DESTINATION include
)

# Export libraries and includes
ament_export_targets(multi_drone_slung_load_cppTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp tf2_ros tf2_eigen Eigen3 std_msgs geometry_msgs tf2_geometry_msgs px4_msgs multi_drone_slung_load_interfaces)
ament_export_include_directories(include)

#ament_export_include_directories(include)
#ament_export_targets(utils State frame_transforms HAS_LIBRARY_TARGET)


# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install data (note without this get "unable to open file for logging" error)
# install(DIRECTORY
#     data
#     DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
