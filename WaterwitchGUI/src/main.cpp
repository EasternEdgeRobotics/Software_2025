#include <GLFW/glfw3.h>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <iostream>
#include "Config.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>

using json = nlohmann::json;
using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;

Config config;

bool showConfigWindow = false;
bool showCameraWindow = false;
bool showAboutWindow = true;

int main() {
    filesystem::create_directory("configs");

    if (!glfwInit()) return -1;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_MAXIMIZED, true);
    
    GLFWwindow* window = glfwCreateWindow(800, 800, "Eastern Edge Waterwitch GUI", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        //top menu bar
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("Cameras")) {
                if (ImGui::MenuItem("Open Camera Window")) {
                    showCameraWindow = true;
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Config")) {
                if (ImGui::MenuItem("Open Config Editor")) {
                    showConfigWindow = true;
                }
                if (ImGui::BeginMenu("Load Config")) {
                    for (const auto& configFile : recursive_directory_iterator("configs")) {
                        if (ImGui::MenuItem(configFile.path().filename().string().c_str())) {
                            ifstream configJson(string("configs/") + configFile.path().filename().string());
                            json configData = json::parse(configJson);
                            std::strncpy(config.cam1ip, configData["cameraIPs"][0].get<std::string>().c_str(), sizeof(config.cam1ip) - 1);
                            std::strncpy(config.cam2ip, configData["cameraIPs"][1].get<std::string>().c_str(), sizeof(config.cam2ip) - 1);
                            std::strncpy(config.cam3ip, configData["cameraIPs"][2].get<std::string>().c_str(), sizeof(config.cam3ip) - 1);
                            config.cam1ip[sizeof(config.cam1ip) - 1] = '\0';
                            config.cam2ip[sizeof(config.cam2ip) - 1] = '\0';
                            config.cam3ip[sizeof(config.cam3ip) - 1] = '\0';
                            std::vector<int> buttons = configData["buttons"].get<std::vector<int>>();
                            config.buttonActions.clear();
                            for (int i = 0; i < buttons.size(); i++) {
                                config.buttonActions.push_back(static_cast<ButtonAction>(buttons[i]));
                            }
                            std::vector<int> axes = configData["axes"].get<std::vector<int>>();
                            config.axisActions.clear();
                            for (int i = 0; i < axes.size(); i++) {
                                config.axisActions.push_back(static_cast<AxisAction>(axes[i]));
                            }
                        }
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();                
            }
            if (ImGui::BeginMenu("Pilot")) {
                if (ImGui::MenuItem("Open Piloting Menu")) {
                    //open pilot menu
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        //config window
        if (showConfigWindow) {
            ImGui::Begin("Config Editor", &showConfigWindow);
            if (ImGui::BeginTabBar("Config Tabs")) {
                if (ImGui::BeginTabItem("Cameras")) {
                    ImGui::Text("Camera 1 URL");
                    ImGui::SameLine(); 
                    ImGui::InputText("##camera1", config.cam1ip, 64);
                    ImGui::Text("Camera 2 URL");
                    ImGui::SameLine(); 
                    ImGui::InputText("##camera2", config.cam2ip, 64);
                    ImGui::Text("Camera 3 URL");
                    ImGui::SameLine(); 
                    ImGui::InputText("##camera3", config.cam3ip, 64);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Controls")) {
                    if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
                        int buttonCount, axisCount;
                        const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttonCount);
                        const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axisCount);
                        if (buttonCount > 0 && config.buttonActions.empty()) config.buttonActions = vector<ButtonAction>(buttonCount, ButtonAction::NONE);
                        if (axisCount > 0 && config.axisActions.empty()) config.axisActions = vector<AxisAction>(axisCount, AxisAction::NONE);
                        ImGui::Text("Connected Controller: %s", glfwGetJoystickName(GLFW_JOYSTICK_1));
                        ImGui::Text("Axis Deadzone");
                        ImGui::SameLine();
                        ImGui::SliderFloat("##deadzone", &config.deadzone, 0.f, 1.f);
                        ImGui::SeparatorText("Buttons");
                        for (int i = 0; i < buttonCount; i++) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, !buttons[i], 1, 1));
                            ImGui::Text((std::string("Button ") + std::to_string(i)).c_str());
                            ImGui::PopStyleColor();
                            ImGui::SameLine(); 
                            if (ImGui::Combo((std::string("##button") + std::to_string(i)).c_str(), reinterpret_cast<int*>(&config.buttonActions[i]), buttonActionLabels, static_cast<int>(ButtonAction::SIZE))) {

                            }
                        }
                        ImGui::SeparatorText("Axes");
                        for (int i = 0; i < axisCount; i++) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1-abs(axes[i]), 1, 1)); //TODO: this is kind of ugly
                            ImGui::Text((std::string("Axis ") + std::to_string(i)).c_str());
                            ImGui::PopStyleColor();
                            ImGui::SameLine();
                            if (ImGui::Combo((std::string("##axis") + std::to_string(i)).c_str(), reinterpret_cast<int*>(&config.axisActions[i]), axisActionLabels, static_cast<int>(AxisAction::SIZE))) {
                                
                            }
                        }
                    } else {
                        ImGui::Text("No Controller Connected.");
                    }
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Save")) {
                    ImGui::Text("Config Name");
                    ImGui::SameLine(); 
                    ImGui::InputText("##configName", config.name, 64);
                    if (!glfwJoystickPresent(GLFW_JOYSTICK_1)) {
                        ImGui::Text("Controller must be connected");
                    } else if (ImGui::Button("Save")) {
                        json configJson;
                        configJson["name"] = config.name;
                        configJson["cameraIPs"][0] = config.cam1ip;
                        configJson["cameraIPs"][1] = config.cam2ip;
                        configJson["cameraIPs"][2] = config.cam3ip;
                        configJson["controller"] = glfwGetJoystickName(GLFW_JOYSTICK_1);
                        configJson["deadzone"] = config.deadzone;
                        for (int i = 0; i < config.buttonActions.size(); i++) {
                            configJson["buttons"][i] = config.buttonActions[i];
                        }
                        for (int i = 0; i < config.axisActions.size(); i++) {
                            configJson["axes"][i] = config.axisActions[i];
                        }
                        std::ofstream ofs;
                        ofs.open("configs/" + string(config.name) + ".json", std::ofstream::out | std::ofstream::trunc);
                        ofs << configJson.dump();
                        ofs.close();
                    }
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            ImGui::End();
        }

        //camera window
        if (showCameraWindow) {
            ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, (io.DisplaySize.y - ImGui::GetFrameHeight())));
            ImGui::SetNextWindowPos(ImVec2{0, ImGui::GetFrameHeight()});
            ImGui::Begin("Camera Window", &showCameraWindow, ImGuiWindowFlags_NoResize);
            ImGui::End();
        }

        glClear(GL_COLOR_BUFFER_BIT);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}