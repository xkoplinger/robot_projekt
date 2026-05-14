add_library(lidar_lib STATIC Lidar.cpp)

target_include_directories(lidar_lib PRIVATE ${OpenCV_INCLUDE_DIRS})
