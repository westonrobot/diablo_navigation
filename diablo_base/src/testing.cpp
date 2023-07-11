#include <iostream>
#include <chrono>
#include "ros/ros.h"

// ros::Rate loop_rate(100);

int main() {
    // Start the timer
    

    // Loop for a specified number of iterations
    int iterations = 1000;
    int i = 0;
    while (i < iterations) {
        auto startTime = std::chrono::high_resolution_clock::now();
        // Perform some task within the loop
        // ...
        std::cout << "Sleeping for some time" << std::endl;
        std::cout << "Sleeping for some time" << std::endl;
        std::cout << "Sleeping for some time" << std::endl;
        std::cout << "     " << std::endl;
        // loop_rate.sleep();
        // Calculate the time taken for each iteration
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = (std::chrono::duration_cast<std::chrono::microseconds>(currentTime - startTime).count() ) / 1000000.00;

        std::cout << "Iteration " << i + 1 << ": " << duration << " seconds" << std::endl;

        // Update the loop counter
        i++;
    }

    return 0;
}