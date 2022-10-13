project(usv_ros)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/HEAR_ROS)

include(hear_ros)


add_subdirectory(src/lib/HEAR2)
add_subdirectory(src/drivers/navio_drivers)

include_directories(
    include
    ${HEAR_ROS_INCLUDE_DIR}    
    ${hear_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${navio_drivers_INCLUDE_DIRS}
)
