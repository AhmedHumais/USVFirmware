cmake_minimum_required(VERSION 2.8.10)

project(navio_drivers C CXX)

include_directories(
    ${usv_arch_INCLUDE_DIRS}
    Navio2/C++/Navio
    include
)

file(GLOB NAVIO2_SRCs Navio2/C++/Navio/Navio2/*.cpp)
file(GLOB NAVIO2_HARDWARE_SRCs Navio2/C++/Navio/Common/*.cpp)

file(GLOB NAVIO_DRIVER_SRCs src/*.cpp)

add_library(${PROJECT_NAME} STATIC ${NAVIO2_HARDWARE_SRCs} ${NAVIO2_SRCs} ${NAVIO_DRIVER_SRCs})

set_target_properties(${PROJECT_NAME}
                      PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} usv_arch)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/Navio2/C++/Navio ${usv_arch_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)