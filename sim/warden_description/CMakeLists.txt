cmake_minimum_required(VERSION 3.8)
project(warden_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models launch rviz config gazebo media maps worlds materials
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
