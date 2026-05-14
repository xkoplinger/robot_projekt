# Pridáme podpriecinky, ktoré vyrobia knižnice
add_subdirectory(environment)
add_subdirectory(lidar)
add_subdirectory(robot)

# Vytvoríme hlavný program z main.cpp
add_executable(robot_sim main.cpp)

# Prepojíme program s našimi modulmi a externými knižnicami
target_link_libraries(robot_sim 
    environment_lib 
    lidar_lib 
    robot_lib
    ${OpenCV_LIBS}
    Threads::Threads
)
