#include <iostream>
#include <cassert>
#include "environment/Environment.h"

int main() {
    // neexistuje subor
    std::cout << "Test 1: Loading non-existent map... ";
    try {
        environment::Config cfg;
        cfg.map_filename = "blbost.png";
        cfg.resolution = 0.05;
        environment::Environment env(cfg);
        std::cout << "FAILED (No exception thrown)" << std::endl;
        return 1;
    } catch (const std::runtime_error& e) {
        std::cout << "PASSED (Caught: " << e.what() << ")" << std::endl;
    }

    // suradnice mimo mapy
    std::cout << "Test 2: Out of bounds coordinates... ";
    try {
        environment::Config cfg;
        cfg.map_filename = "map/opk-map.png";
        cfg.resolution = 0.05;
        environment::Environment env(cfg);

        // bod mimo mapy je occupied, stena
        if (env.isOccupied(999.0, 999.0) == true && env.isOccupied(-1.0, -1.0) == true) {
            std::cout << "PASSED" << std::endl;
        } else {
            std::cout << "FAILED" << std::endl;
            return 1;
        }
    } catch (...) {
        std::cout << "FAILED (Exception during test)" << std::endl;
        return 1;
    }

    std::cout << "\nALL UNIT TESTS PASSED!" << std::endl;
    return 0;
}
