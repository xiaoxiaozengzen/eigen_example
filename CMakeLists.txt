cmake_minimum_required(VERSION 3.14)
project(eigen_example)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(BUILD_DEBUG)
    add_definitions("-g")
endif()

find_package(Eigen3 REQUIRED)
message(STATUS "EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}")

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_exe ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(${PROJECT_NAME}_exe Eigen3::Eigen)

add_executable(${PROJECT_NAME}_matrix_exe ${CMAKE_CURRENT_SOURCE_DIR}/src/matrix.cpp)
target_link_libraries(${PROJECT_NAME}_matrix_exe Eigen3::Eigen)

add_executable(${PROJECT_NAME}_array_exe ${CMAKE_CURRENT_SOURCE_DIR}/src/array.cpp)
target_link_libraries(${PROJECT_NAME}_array_exe Eigen3::Eigen)

add_executable(${PROJECT_NAME}_transformer_exe ${CMAKE_CURRENT_SOURCE_DIR}/src/transformer.cpp)
target_link_libraries(${PROJECT_NAME}_transformer_exe Eigen3::Eigen)

add_executable(${PROJECT_NAME}_rfu2ego_exe ${CMAKE_CURRENT_SOURCE_DIR}/src/rfu2ego.cpp)
target_link_libraries(${PROJECT_NAME}_rfu2ego_exe Eigen3::Eigen)

add_executable(${PROJECT_NAME}_realusing ${CMAKE_CURRENT_SOURCE_DIR}/src/realusing.cpp)
target_link_libraries(${PROJECT_NAME}_realusing Eigen3::Eigen ${${nolhmann_json_LIBRARIES}})
target_include_directories(${PROJECT_NAME}_realusing PRIVATE ${nolhmann_json_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_affine3d ${CMAKE_CURRENT_SOURCE_DIR}/src/affined.cpp)
target_link_libraries(${PROJECT_NAME}_affine3d Eigen3::Eigen)