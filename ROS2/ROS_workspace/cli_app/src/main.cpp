// main.cpp
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <mutex>

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

    //main loop for the CLI
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "exit") {
            break;
        } else {
            handleCommand(input);
        }
    }

    // Exit message
    std::cout << "Goodbye!" << std::endl;

    //cleanup and exit
    running.store(false);
    return 0;
}