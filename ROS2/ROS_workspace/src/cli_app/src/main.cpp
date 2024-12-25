#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <mutex>
#include <chrono>
#include "gui.h"
#include "cli.h"

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

    // Initialize ROS 2
    rclcpp::init(0, nullptr);

    // Create a ROS 2 node for controller input
    auto node = rclcpp::Node::make_shared("controller_input_publisher");
    auto controller_input_publisher = node->create_publisher<eer_messages::msg::PilotInput>("/PilotInput", 10);

    std::thread ros_spin_thread([&]() {
        rclcpp::spin(node);
    });

    // Check for command line launch options
    //TODO: Move to CLI
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            printCLIOptions(arg);
            printCLICommands();
            return 0;
        } else if (arg == "launch_gui") {
            std::thread guiThread(launchGUI, controller_input_publisher, false);
            guiThread.detach();

        }else if (arg == "launch_controller") {
            std::thread controlThread(launchGUI, controller_input_publisher, true);
            controlThread.detach();

        }
        
    }

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
            if (input == "0" || input == "pilot") {
                std::cout << "Piloting the robot..." << std::endl;
            } else if (input == "1" || input == "controller") {
                std::cout << "Selecting the controller..." << std::endl;
            } else if (input == "2" || input == "make_profile") {
                std::cout << "Making a control profile..." << std::endl;
            } else if (input == "3" || input == "set_stream_urls") {
                CameraStreamUrls(true);
            } else if (input == "4" || input == "help") {
                printCLICommands();
            } else if (input == "5" || input == "version") {
                std::cout << "EER CLI version 1.0" << std::endl;
            } else if (input == "6" || input == "exit") {
                std::cout << "Exiting the application..." << std::endl;
            exit(0);
            } else if (input == "7" || input == "launch_gui") {
                std::thread guiThread(launchGUI, controller_input_publisher, false);
                guiThread.detach();
            } else if (input == "8" || input == "launch_controller_only") {
                std::thread controlThread(launchGUI, controller_input_publisher, true);
                controlThread.detach();
            } else {
                std::cout << "Unknown command: " << input << std::endl;
                std::cout << "Type 'help' to see available commands." << std::endl;
            }
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
    ros_spin_thread.join();

    return 0;
}