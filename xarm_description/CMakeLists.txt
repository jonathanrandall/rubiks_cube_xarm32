cmake_minimum_required(VERSION 3.5)
project(xarm_description)

find_package(ament_cmake REQUIRED)



install(
  DIRECTORY xarmbot/meshes xarmbot/urdf xarmbot/rviz
  DESTINATION share/${PROJECT_NAME}/xarmbot
)

ament_package()
