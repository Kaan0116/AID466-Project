cmake_minimum_required(VERSION 3.8)
project(following_study)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)

# Python package (geometry, study_config, nodes)
ament_python_install_package(${PROJECT_NAME})

# ROS nodes (ros2 run following_study <name>)
install(PROGRAMS following_study/follower_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME follower_node)
install(PROGRAMS following_study/person_driver.py
  DESTINATION lib/${PROJECT_NAME} RENAME person_driver)

# Study scripts (ros2 run following_study <script>.py)
install(PROGRAMS
  scripts/capture_stills.py
  scripts/record_clip.py
  scripts/evaluate_system.py
  scripts/demo_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

# xacro -> URDF at build time (xacro is REQUIRED above, so this always runs)
find_program(XACRO_EXEC xacro REQUIRED)
set(URDF_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/robot.urdf)
add_custom_command(
  OUTPUT ${URDF_OUTPUT}
  COMMAND ${XACRO_EXEC} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.urdf.xacro -o ${URDF_OUTPUT}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.urdf.xacro
  COMMENT "xacro -> robot.urdf"
)
add_custom_target(generate_urdf ALL DEPENDS ${URDF_OUTPUT})
install(FILES ${URDF_OUTPUT} DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY worlds models launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
