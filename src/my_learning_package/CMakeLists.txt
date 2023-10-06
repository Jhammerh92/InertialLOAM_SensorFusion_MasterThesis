cmake_minimum_required(VERSION 3.5)
project(my_learning_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(ndt_omp_ros2 REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pclomp REQUIRED)
# uncomment the following section in order to fill in

# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies "std_msgs" "std_srvs" "rclcpp" "geometry_msgs" "sensor_msgs" "tf2" "tf2_geometry_msgs" "sensor_msgs" "pcl_conversions" "pcl_ros" "nav_msgs")

# add_executable(hello_publisher src/hello_publisher.cpp)
# ament_target_dependencies(hello_publisher ${dependencies})

# add_executable(hello_subscriber src/hello_subscriber.cpp)
# ament_target_dependencies(hello_subscriber ${dependencies})

# add_executable(imu_listener src/imu_listener.cpp)
# ament_target_dependencies(imu_listener ${dependencies})

# add_executable(imu_calibrater src/imu_calibrater.cpp)
# ament_target_dependencies(imu_calibrater ${dependencies})

# add_executable(imu_dead_recon src/imu_dead_recon.cpp)
# ament_target_dependencies(imu_dead_recon ${dependencies})

# add_executable(imu_tf_broadcaster src/imu_tf_broadcaster.cpp)
# ament_target_dependencies(imu_tf_broadcaster ${dependencies})

add_executable(lidar_odometry src/LidarOdometry.cpp)
ament_target_dependencies(lidar_odometry ${dependencies})

add_executable(preprocessing src/Preprocessing.cpp)
ament_target_dependencies(preprocessing ${dependencies})

add_executable(loam_backend src/Backend.cpp)
ament_target_dependencies(loam_backend ${dependencies})

add_executable(ekf src/EKF.cpp)
ament_target_dependencies(ekf ${dependencies})

add_executable(ekf_ins src/EKF_INS.cpp)
ament_target_dependencies(ekf_ins ${dependencies})

add_executable(imu_filter src/IMUFilter.cpp)
ament_target_dependencies(imu_filter ${dependencies})

add_executable(ins src/INS.cpp)
ament_target_dependencies(ins ${dependencies})

# add_executable(format_convert src/FormatConvert.cpp)
# ament_target_dependencies(format_convert ${dependencies})

# velodyne files
add_executable(preprocessing_velodyne src/PreprocessingVelodyne.cpp)
ament_target_dependencies(preprocessing_velodyne ${dependencies})

add_executable(lidar_odometry_velodyne src/LidarOdometryVelodyne.cpp)
ament_target_dependencies(lidar_odometry_velodyne ${dependencies})

add_executable(static_recorder src/StaticRecord.cpp)
ament_target_dependencies(static_recorder ${dependencies})

add_executable(my_test_node src/my_test_node.cpp)

target_include_directories(my_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_include_directories(lidar_odometry PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

install(TARGETS 
  # my_test_node
  # hello_publisher
  # hello_subscriber
  # imu_listener
  # imu_calibrater
  # imu_dead_recon
  # imu_tf_broadcaster
  lidar_odometry
  preprocessing
  loam_backend
  ekf
  ekf_ins
  imu_filter
  ins
  # format_convert
  preprocessing_velodyne
  lidar_odometry_velodyne
  static_recorder
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  rviz
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
