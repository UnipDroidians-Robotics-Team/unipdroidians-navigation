cmake_minimum_required(VERSION 3.8)
project(udh1_gazebo)

find_package(ament_cmake REQUIRED)

# Instala pastas de launch e worlds
install(
  DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
