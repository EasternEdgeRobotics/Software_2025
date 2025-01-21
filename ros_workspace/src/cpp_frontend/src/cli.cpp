#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "gui.h"
#include "eer_interfaces/srv/cameras.hpp"

using namespace std::chrono_literals;

// Function to print colored ASCII image
void printColoredAsciiImage() {
    
    std::string art = R"([49m                                                  [48;5;0m                              [49m                                                  [m
[49m                                        [48;5;0m            [48;5;11m                          [48;5;0m            [49m                                        [m
[49m                                [48;5;0m        [48;5;220m  [48;5;11m                                              [48;5;220m  [48;5;0m        [49m                                [m
[49m                            [48;5;0m      [48;5;11m                                                              [48;5;0m      [49m                            [m
[49m                        [48;5;0m    [48;5;11m                                                                          [48;5;0m    [49m                        [m
[49m                    [48;5;0m    [48;5;11m                                                                                  [48;5;0m    [49m                    [m
[49m                [48;5;0m    [48;5;11m                                                                                          [48;5;0m    [49m                [m
[49m              [48;5;0m    [48;5;11m                                              [48;5;236m  [48;5;103m    [48;5;236m  [48;5;11m                                        [48;5;0m    [49m              [m
[49m            [48;5;0m  [48;5;11m                                                  [48;5;236m  [48;5;15m    [48;5;236m  [48;5;11m                                            [48;5;0m  [49m            [m
[49m          [48;5;0m  [48;5;11m      [48;5;236m            [48;5;60m  [48;5;11m    [48;5;236m        [48;5;220m  [48;5;236m                  [48;5;15m      [48;5;236m                                            [48;5;11m    [48;5;0m  [49m          [m
[49m        [48;5;0m  [48;5;11m        [48;5;236m  [48;5;15m            [48;5;11m  [48;5;60m  [48;5;15m          [48;5;220m  [48;5;15m            [48;5;236m  [48;5;15m          [48;5;236m  [48;5;15m            [48;5;236m  [48;5;15m            [48;5;236m  [48;5;15m            [48;5;236m  [48;5;11m    [48;5;0m  [49m        [m
[49m      [48;5;0m  [48;5;11m          [48;5;236m  [48;5;15m    [48;5;236m  [48;5;15m      [48;5;236m  [48;5;15m            [48;5;184m  [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m    [48;5;15m      [48;5;236m    [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m  [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m  [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m  [48;5;11m      [48;5;0m  [49m      [m
[49m    [48;5;0m  [48;5;11m            [48;5;236m  [48;5;15m          [48;5;61m  [48;5;60m  [48;5;15m            [48;5;236m  [48;5;15m            [48;5;236m    [48;5;15m    [48;5;236m  [48;5;11m  [48;5;137m  [48;5;15m            [48;5;236m  [48;5;15m      [48;5;11m      [48;5;184m  [48;5;15m            [48;5;236m  [48;5;11m        [48;5;0m  [49m    [m
[49m    [48;5;0m  [48;5;11m            [48;5;15m          [48;5;236m  [48;5;11m  [48;5;146m  [48;5;15m          [48;5;236m    [48;5;15m            [48;5;11m  [48;5;15m      [48;5;236m  [48;5;11m  [48;5;236m  [48;5;15m          [48;5;236m    [48;5;15m      [48;5;11m      [48;5;236m  [48;5;15m    [48;5;146m  [48;5;15m      [48;5;11m          [48;5;0m  [49m    [m
[49m  [48;5;0m  [48;5;11m            [48;5;236m  [48;5;15m            [48;5;236m  [48;5;15m            [48;5;236m  [48;5;15m      [48;5;236m  [48;5;15m    [48;5;146m  [48;5;137m  [48;5;15m      [48;5;11m    [48;5;252m  [48;5;15m    [48;5;236m  [48;5;15m    [48;5;252m  [48;5;236m  [48;5;15m    [48;5;236m  [48;5;11m      [48;5;236m  [48;5;15m    [48;5;236m  [48;5;15m      [48;5;11m            [48;5;0m  [49m  [m
[49m  [48;5;0m  [48;5;11m            [48;5;236m  [48;5;15m          [48;5;236m  [48;5;101m  [48;5;15m            [48;5;236m  [48;5;104m  [48;5;15m          [48;5;236m  [48;5;11m  [48;5;15m      [48;5;11m    [48;5;146m  [48;5;15m  [48;5;0m    [48;5;102m  [48;5;15m  [48;5;236m  [48;5;15m      [48;5;60m  [48;5;11m      [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m  [48;5;11m            [48;5;0m  [49m  [m
[48;5;0m  [48;5;58m  [48;5;11m              [48;5;236m          [48;5;11m    [48;5;236m                        [48;5;11m    [48;5;233m  [48;5;0m                [48;5;236m    [48;5;11m  [48;5;236m        [48;5;11m      [48;5;236m                [48;5;11m            [48;5;3m  [48;5;0m  [m
[48;5;0m  [48;5;11m                                                      [48;5;0m    [48;5;15m  [48;5;11m                                                                  [48;5;0m  [m
[48;5;0m  [48;5;11m                                                    [48;5;0m    [48;5;11m                                                                      [48;5;0m  [m
[48;5;0m  [48;5;11m                                                  [48;5;0m    [48;5;11m            [48;5;236m  [48;5;15m    [48;5;236m  [48;5;11m                                                    [48;5;0m  [m
[48;5;0m  [48;5;11m                              [48;5;15m  [48;5;11m                  [48;5;0m  [48;5;15m  [48;5;11m            [48;5;15m      [48;5;236m  [48;5;11m                                                    [48;5;0m  [m
[48;5;0m  [48;5;11m                    [48;5;15m  [48;5;11m      [48;5;227m  [48;5;9m  [48;5;11m                [48;5;15m  [48;5;0m  [48;5;15m  [48;5;11m        [48;5;101m    [48;5;15m      [48;5;143m  [48;5;11m  [48;5;143m            [48;5;179m            [48;5;11m                          [48;5;0m  [m
[48;5;0m  [48;5;58m  [48;5;11m                    [48;5;9m  [48;5;227m  [48;5;11m  [48;5;9m    [48;5;11m                [48;5;0m      [48;5;11m    [48;5;236m  [48;5;15m            [48;5;11m  [48;5;236m  [48;5;15m          [48;5;236m    [48;5;15m            [48;5;11m                      [48;5;3m  [48;5;0m  [m
[49m  [48;5;0m  [48;5;11m                    [48;5;9m    [48;5;15m  [48;5;9m    [48;5;11m      [48;5;15m  [48;5;11m        [48;5;0m      [48;5;230m  [48;5;143m  [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m    [48;5;15m            [48;5;236m    [48;5;15m    [48;5;236m  [48;5;15m      [48;5;11m                      [48;5;0m  [49m  [m
[49m  [48;5;0m  [48;5;11m          [48;5;15m  [48;5;0m      [48;5;15m  [48;5;9m          [48;5;255m  [48;5;9m    [48;5;0m                  [48;5;236m  [48;5;15m      [48;5;0m          [48;5;15m    [48;5;236m  [48;5;15m      [48;5;11m  [48;5;7m  [48;5;15m    [48;5;251m  [48;5;15m    [48;5;60m  [48;5;11m                      [48;5;0m  [49m  [m
[49m    [48;5;0m  [48;5;11m        [48;5;0m          [48;5;15m  [48;5;9m            [48;5;0m        [48;5;251m  [48;5;15m  [48;5;0m        [48;5;236m  [48;5;15m    [48;5;236m  [48;5;0m          [48;5;15m    [48;5;236m  [48;5;15m      [48;5;11m  [48;5;15m          [48;5;60m  [48;5;11m                      [48;5;0m  [49m    [m
[49m    [48;5;0m  [48;5;11m                    [48;5;9m            [48;5;11m          [48;5;0m        [48;5;11m  [48;5;15m      [48;5;236m  [48;5;15m      [48;5;143m  [48;5;15m      [48;5;236m  [48;5;15m    [48;5;236m    [48;5;15m            [48;5;60m  [48;5;11m                    [48;5;0m  [49m    [m
[49m      [48;5;0m  [48;5;11m                [48;5;15m      [48;5;9m    [48;5;11m              [48;5;0m    [48;5;15m  [48;5;11m    [48;5;15m              [48;5;101m  [48;5;15m            [48;5;236m    [48;5;15m          [48;5;236m  [48;5;11m                    [48;5;0m  [49m      [m
[49m        [48;5;0m  [48;5;11m                        [48;5;15m  [48;5;11m            [48;5;0m  [48;5;15m  [48;5;11m      [48;5;236m                      [48;5;15m      [48;5;236m  [48;5;184m  [48;5;236m          [48;5;11m                    [48;5;0m  [49m        [m
[49m          [48;5;0m  [48;5;11m                                  [48;5;248m  [48;5;0m  [48;5;11m                      [48;5;61m        [48;5;15m      [48;5;11m                                [48;5;0m  [49m          [m
[49m            [48;5;0m  [48;5;11m                                [48;5;0m    [48;5;11m                    [48;5;236m  [48;5;15m            [48;5;236m  [48;5;11m                              [48;5;0m  [49m            [m
[49m              [48;5;0m    [48;5;11m                            [48;5;15m  [48;5;0m  [48;5;11m                                                              [48;5;0m    [49m              [m
[49m                [48;5;0m    [48;5;11m                            [48;5;0m    [48;5;11m                                                          [48;5;0m    [49m                [m
[49m                    [48;5;0m    [48;5;11m                          [48;5;0m        [48;5;234m  [48;5;0m        [48;5;15m  [48;5;11m                                    [48;5;0m    [49m                    [m
[49m                        [48;5;0m    [48;5;11m                              [48;5;228m  [48;5;15m    [48;5;0m      [48;5;15m  [48;5;11m                              [48;5;0m    [49m                        [m
[49m                            [48;5;0m      [48;5;11m                                      [48;5;15m  [48;5;11m                      [48;5;0m      [49m                            [m
[49m                                [48;5;0m        [48;5;220m  [48;5;11m                                              [48;5;220m  [48;5;0m        [49m                                [m
[49m                                        [48;5;0m            [48;5;11m                          [48;5;0m            [49m                                        [m
[49m                                                  [48;5;0m                              [49m                                                  [m
)";

std::cout << art << std::endl;
}
void printIntroduction() {
    std::string introduction = R"(EER is a CLI application for Eastern Edge Robotics)";
    std::cout << introduction << std::endl;
}

void printCLIOptions(const std::string& arg) {
    if (arg == "-h" || arg == "--help") {
        std::string options = R"(Options:
        -h, --help      show this help message and exit)";
        std::cout << options << std::endl;
    }
}

std::vector<std::string> CameraStreamUrls(bool set_urls = false) {

    std::vector<std::string> camera_urls;

    // Create a ROS 2 node and publisher for camera urls
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("camera_urls_client");
    rclcpp::Client<eer_interfaces::srv::Cameras>::SharedPtr camera_urls_client =
        node->create_client<eer_interfaces::srv::Cameras>("/camera_urls");
    
    // Wait for the service server to become available
    if (!camera_urls_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return camera_urls;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Count not fetch camera URLs from database. /camera_urls service is not available.");
        return camera_urls;
    }

    // Get the current camera urls
    auto request = std::make_shared<eer_interfaces::srv::Cameras::Request>();
    request->state = 1; // We are just looking to read the current camera urls for now

    auto result = camera_urls_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {   
        auto response = result.get();
        camera_urls = response->camera_urls;
        if (!set_urls) return camera_urls; 
        else std::cout << "The current camera urls are: " << response->camera_urls_json << std::endl;

    } else {
        std::cout << "Failed to get camera URLs." << std::endl;
        return camera_urls;
    }

    while (true) {

        std::cout << "Which url would you like to change? (type an integer, see_urls, or done): " << std::endl;

        std::string input;
        std::cout << ">> ";
        std::getline(std::cin, input);

        if (input == "see_urls") {
            for (size_t i = 0; i < camera_urls.size(); ++i) {
            std::cout << "Camera " << i << " URL: " << camera_urls[i] << std::endl;
            }
            continue;
        } else if (input == "done") {
            break;
        }

        int index;
        try {
            index = std::stoi(input);
        } catch (const std::invalid_argument& e) {
            std::cout << "Invalid input." << std::endl;
            continue;
        }

        if (index >= 0 && index < static_cast<int>(camera_urls.size())) {
            std::cout << "Enter new URL for camera " << index << ": " << std::endl;
            std::cout << ">> "; 
            std::getline(std::cin, input);
            camera_urls[index] = input;
        } else {
            std::cout << "Invalid input." << std::endl;
        }
    }

    request->state = 0; // We are looking to set the camera urls now
    request->camera_urls = camera_urls;

    result = camera_urls_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {  
        auto response = result.get();
        if (response->success) std::cout << "Camera URLs set!" << std::endl;
        return camera_urls;
    } 
    
    std::cout << "Failed to set camera URLs." << std::endl;

    return camera_urls;
}

void printCLICommands() {
    std::string commands = R"(Commands:
        0: pilot                  Pilot the robot using the selected controller
        1: controller             Select the desired controller
        2: make_profile           Make a control profile
        3: set_stream_urls        Set the camera stream URLs
        4: help                   Display possible commands and usage
        5: version                Display the current version of the tool
        6: exit                   Exit the application
        7: launch_gui             Launch the GUI application)";
    std::cout << commands << std::endl;
}