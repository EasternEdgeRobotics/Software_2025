#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <mutex>
#include <chrono>
#include "gui.h"

// Forward declarations
void printCLIOptions(const std::string& arg);
void printCLICommands();
void handleCommand(const std::string& command);
void printColoredAsciiImage();
void printIntroduction();

// Atomic bool to control the main program loop
std::atomic<bool> running(true);

void signalHandler(int signum) {
    (void)signum; // Avoid unused parameter warning
    running.store(false);
}

int main(int argc, char* argv[]) {

    // Call the signal handler function when user does CTRL+C
    std::signal(SIGINT, signalHandler);

    // Check for command line launch options
    //TODO: Move to CLI
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            printCLIOptions(arg);
            printCLICommands();
            return 0;
        } else if (arg == "launch_gui") {
            std::thread guiThread(launchGUI, false);
            guiThread.detach();

        }else if (arg == "launch_controller") {
            std::thread controlThread(launchGUI, true);
            controlThread.detach();

        }
        
    }

    // Initialize ROS 2
    rclcpp::init(0, nullptr);

    printColoredAsciiImage();
    printIntroduction();

    const std::chrono::milliseconds loop_duration(10); // 100 Hz -> 10 ms per loop iteration

    //main loop for the CLI
    while (running.load()) {
        auto loop_start_time = std::chrono::steady_clock::now();

        std::string input;
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "exit") {
            running.store(false);
        } else {
            handleCommand(input);
        }
        
        // Sleep to maintain 100 Hz loop rate
        auto loop_end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
        if (elapsed_time < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed_time);
        }
    }

    // Exit message
    std::cout << "Goodbye!" << std::endl;
    
    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}