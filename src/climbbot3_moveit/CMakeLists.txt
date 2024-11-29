cmake_minimum_required(VERSION 3.22)
project(climbbot3_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/fake_points.py
  scripts/climb.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})