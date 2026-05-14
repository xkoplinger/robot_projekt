add_library(robot_lib STATIC Robot.cpp)
target_link_libraries(robot_lib Threads::Threads)

target_include_directories(robot_lib PRIVATE ${OpenCV_INCLUDE_DIRS})
