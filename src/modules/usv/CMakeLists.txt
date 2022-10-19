cmake_minimum_required(VERSION 2.8.10)

project(usv C CXX)

include_directories(
    ${usv_arch_INCLUDE_DIRS}
    ${navio_drivers_INCLUDE_DIRS}
    include
)

file(GLOB USV_SRCs src/*.cpp)

add_library(${PROJECT_NAME} STATIC ${USV_SRCs})

set_target_properties(${PROJECT_NAME}
                      PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} usv_arch navio_drivers)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${usv_arch_INCLUDE_DIRS} ${navio_drivers_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)