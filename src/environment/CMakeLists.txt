add_library(environment_lib STATIC Environment.cpp)
target_link_libraries(environment_lib ${OpenCV_LIBS})
