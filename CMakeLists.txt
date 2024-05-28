cmake_minimum_required(VERSION 3.25.2)

project(metamorphosis-project)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find and configure PCL
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find and configure nlohmann JSON
add_subdirectory(external/json)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/json/single_include)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

