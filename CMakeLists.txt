cmake_minimum_required(VERSION 3.8)
project(following_study)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)

# Python paketi kur
ament_python_install_package(${PROJECT_NAME})

# Python yürütülebilir dosyaları (node'lar)
install(PROGRAMS
  following_study/follower_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME follower_node
)

# Python scriptleri
install(PROGRAMS
  scripts/capture_stills.py
  scripts/record_clip.py
  scripts/evaluate_system.py
  DESTINATION lib/${PROJECT_NAME}
)

# URDF dosyasını xacro'dan üret
find_program(XACRO_EXEC xacro)
if(XACRO_EXEC)
  set(URDF_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/robot.urdf)
  add_custom_command(
    OUTPUT ${URDF_OUTPUT}
    COMMAND ${XACRO_EXEC} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.urdf.xacro -o ${URDF_OUTPUT}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.urdf.xacro
    COMMENT "xacro -> URDF dönüştürülüyor"
  )
  add_custom_target(generate_urdf ALL DEPENDS ${URDF_OUTPUT})
  install(FILES ${URDF_OUTPUT} DESTINATION share/${PROJECT_NAME}/urdf)
endif()

# Statik dosyalar
install(DIRECTORY
  worlds
  models
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
