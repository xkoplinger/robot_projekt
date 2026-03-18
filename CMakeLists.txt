cmake_minimum_required(VERSION 3.10)
project(RobotSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. najst opencv
find_package(OpenCV REQUIRED)

# 2. include hlavičky
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# 3. vytvorenie kniznice
add_library(robot_logic
    src/environment/Environment.cpp
    src/lidar/Lidar.cpp
    src/robot/Robot.cpp
)

# prepojenie kniznice s opencv
target_link_libraries(robot_logic ${OpenCV_LIBS})

# 4. vytvorenie spustitelnej aplikacie
add_executable(robot_sim src/main.cpp)

# prepojenie aplikacie s nasou kniznicou
target_link_libraries(robot_sim robot_logic)

# vytvorenie unittestu
add_executable(unit_test src/test_env.cpp)
target_link_libraries(unit_test robot_logic)
