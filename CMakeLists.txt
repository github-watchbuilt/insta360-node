cmake_minimum_required(VERSION 3.8)
project(insta360_ros_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  tf2_ros::tf2_ros
  include
)


link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/include
)


# Add executables
add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs cv_bridge OpenCV)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  CameraSDK
  ${AVCODEC_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  tf2_ros::tf2_ros
)


# Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  scripts/live_processing.py
  scripts/get_images.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY lib/ DESTINATION lib)

ament_package()
