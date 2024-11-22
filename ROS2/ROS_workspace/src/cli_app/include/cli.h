#ifndef CLI_H
#define CLI_H

#include <string>
#include <vector>

// Function declarations
void printColoredAsciiImage();
void handleCommand(const std::string&);
std::vector<std::string> CameraStreamUrls(bool set_urls = false);

#endif // CLI_H