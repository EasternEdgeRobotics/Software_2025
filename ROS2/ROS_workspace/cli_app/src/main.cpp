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

int main(int argc, char* argv[]) {
    std::atomic<bool> running(true);

    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            printCLIOptions(arg);
            printCLICommands();
            return 0;
        } else if (arg == "launch_gui") {
            std::thread guiThread(launchGUI);
            guiThread.detach();

        }
    }

    std::string input;

    printColoredAsciiImage();
    printIntroduction();

    while (true) {
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "exit") {
            break;
        } else {
            handleCommand(input);
        }
    }

    std::cout << "Goodbye!" << std::endl;

    running.store(false);
    return 0;
}