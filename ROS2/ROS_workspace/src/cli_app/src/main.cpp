// main.cpp
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <chrono>
//
// Forward declarations
void launchGUI();
void printCLIOptions(const std::string& arg);
void printCLICommands();
void handleCommand(const std::string& command);
void printColoredAsciiImage();
void printIntroduction();
void launchController();

int main(int argc, char* argv[]) {

    //atomic bool to keep track of the main loop
    std::atomic<bool> running(true);

    // Check for command line launch options
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            printCLIOptions(arg);
            printCLICommands();
            return 0;
        } else if (arg == "launch_gui") {
            std::thread guiThread(launchGUI);
            guiThread.detach();

        }else if (arg == "launch_controller") {
            std::thread controlThread(launchController);
            controlThread.detach();

        }
        
    }

    std::string input;

    printColoredAsciiImage();
    printIntroduction();

    const std::chrono::milliseconds loop_duration(10); // 100 Hz -> 10 ms per loop iteration

    //main loop for the CLI
    while (true) {
        auto loop_start_time = std::chrono::steady_clock::now();

        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "exit") {
            break;
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

    //cleanup and exit
    running.store(false);
    return 0;
}