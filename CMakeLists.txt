cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0046 OLD)

project(usv_firmware)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/HEAR_ROS)

add_subdirectory(src/lib/USV_Arch)
add_subdirectory(src/drivers/navio_drivers)
add_subdirectory(src/modules/usv)

include(hear_ros)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    std_srvs
    std_msgs
    roscpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nodelet
)

include_directories(
  ${usv_arch_INCLUDE_DIRS}
  ${navio_drivers_INCLUDE_DIRS}
  ${HEAR_ROS_INCLUDE_DIR}
  ${usv_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} ${HEAR_ROS_SRCs})
target_link_libraries(${PROJECT_NAME} usv_arch)

add_executable(usv_node src/usv_node.cpp)
add_dependencies(usv_node ${catkin_LIBRARIES})
target_link_libraries(usv_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(state_est_node src/state_est_node.cpp)
add_dependencies(state_est_node ${catkin_LIBRARIES})
target_link_libraries(state_est_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(state_estimator_nodelet src/StateEstimatorNodelet.cpp)
add_dependencies(state_estimator_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_estimator_nodelet
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    usv_arch 
    navio_drivers
)

add_library(usv_nodelet src/UsvNodelet.cpp)
add_dependencies(usv_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(usv_nodelet
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    usv_arch 
    navio_drivers
    usv
)

install(TARGETS usv_nodelet state_estimator_nodelet ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(DIRECTORY include/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES usv_nodelet.xml state_estimator_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
