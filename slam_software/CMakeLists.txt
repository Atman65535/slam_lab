cmake_minimum_required(VERSION 4.2)

set(PROJECT_NAME slam_software)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_BUILD_TYPE Debug)

project(${PROJECT_NAME})

find_package(PCL REQUIRED)
message(${PCL_INCLUDE_DIRS})
message(${PCL_LIBRARIES})

find_package(Eigen3  REQUIRED)
find_package(nlohmann_json REQUIRED)

# find_package(Sophus REQUIRED)


include_directories(./inc/)

add_library(common
  ./src/so3_math.cpp
)
target_include_directories(common
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(common INTERFACE
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  nlohmann_json::nlohmann_json
)


add_executable(${PROJECT_NAME} ./src/main.cpp)
add_executable(stream_read ./tools/stream_read.cpp)
add_executable(test ./tools/test.cpp
  ./src/so3_math.cpp
)
add_executable(test2 ./tools/test2.cpp)
add_executable(test3 ./tools/test3.cpp)


# target_compile_options(${PROJECT_NAME} PRIVATE -pthread)
# target_link_options(${PROJECT_NAME} PRIVATE -pthread -sPTHREAD_POOL_SIZE=8)

target_link_libraries(${PROJECT_NAME} common)
target_link_libraries(stream_read
  common
)

target_link_libraries(test common)
target_link_libraries(test2 common)
target_link_libraries(test3 common)
