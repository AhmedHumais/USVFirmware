cmake_minimum_required(VERSION 3.5)
project(USVFirmware)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/HEAR_ROS2)

add_subdirectory(src/lib/USV_Arch)
add_subdirectory(src/drivers/navio_drivers)
add_subdirectory(src/modules/usv)

include(hear_ros2)

include_directories(
  ${usv_arch_INCLUDE_DIRS}
  ${navio_drivers_INCLUDE_DIRS}
  ${HEAR_ROS2_INCLUDE_DIR}
  include
)

add_library(${PROJECT_NAME} ${HEAR_ROS2_SRCs})
ament_target_dependencies(${PROJECT_NAME} ${HEAR_ROS2_AMENT_DEPS})
if(EXISTS ${HEAR_ROS2_DEFs})
  target_compile_definitions(${PROJECT_NAME} ${HEAR_ROS2_DEFs})
endif()
target_link_libraries(${PROJECT_NAME} usv_arch)


add_executable(state_est_node src/StateEstimatorNodelet.cpp)
ament_target_dependencies(state_est_node
  ${HEAR_ROS2_AMENT_DEPS}
  usv_arch
  ${PROJECT_NAME}
  navio_drivers
)
target_link_libraries(state_est_node usv_arch navio_drivers ${PROJECT_NAME})

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

install(TARGETS
  state_est_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
