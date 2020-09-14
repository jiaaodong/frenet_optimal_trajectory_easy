cmake_minimum_required(VERSION 3.10)
project(FrenetOptimalTrajectory_demo)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-march=native -O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")


find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Eigen3)

if(Eigen3_FOUND)
    set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen)
    set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(.)
include_directories(
        ${CUSTOM_EIGEN}
)

include_directories("${CMAKE_SOURCE_DIR}/include")
link_directories("${CMAKE_SOURCE_DIR}/lib")

add_executable(FrenetOptimalTrajectory_demo
               main.cpp)
target_link_libraries(FrenetOptimalTrajectory_demo Eigen3::Eigen Qt5::Widgets Qt5::Gui FrenetOptimalTrajectory)
