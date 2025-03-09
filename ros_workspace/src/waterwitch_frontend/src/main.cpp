#include "Image.hpp"
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <nlohmann/json.hpp>
#include "Config.hpp"
#include "ROS.hpp"
#include "Camera.hpp"
#include <iostream>
#include <memory>
#include "images/logo.h"
#include "images/nosignal.h"

using json = nlohmann::json;

UserConfig user_config;
WaterwitchConfig waterwitch_config;
Power power;

bool showConfigWindow = false;
bool showCameraWindow = false;
bool showPilotWindow = false;

vector<string> names;
vector<string> configs;
int configuration_mode_thruster_number = 0;
bool configuration_mode = false;

int main(int argc, char **argv) {
    //initialize glfw, imgui, and rclcpp (ros)    
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_MAXIMIZED, true);
    GLFWwindow* window = glfwCreateWindow(800, 800, "Eastern Edge Waterwitch GUI", NULL, NULL);
    if (!window) {
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
    rclcpp::init(argc, argv);
    
    //create images
    unsigned int eerLogo = loadEmbeddedTexture(logo_png, logo_png_len);
    unsigned int noSignal = loadEmbeddedTexture(nosignal_jpg, nosignal_jpg_len);

    //create ros nodes
    auto saveConfigNode = std::make_shared<SaveConfigPublisher>();
    auto pilotInputNode = std::make_shared<PilotInputPublisher>();

    //get configs from ros, then set the names and configs
    std::array<std::vector<std::string>, 2> configRes = getConfigs();
    names = configRes[0];
    configs = configRes[1];

    Camera cam1(user_config.cam1ip, noSignal);
    Camera cam2(user_config.cam2ip, noSignal);
    Camera cam3(user_config.cam3ip, noSignal);

    cam1.start();
    cam2.start();
    cam3.start();

    //render loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        //controller loop
        if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
            int buttonCount, axisCount;
            const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttonCount);
            const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axisCount);
            if (buttonCount > 0 && user_config.buttonActions.empty()) user_config.buttonActions = vector<ButtonAction>(buttonCount, ButtonAction::NONE);
            if (axisCount > 0 && user_config.axisActions.empty()) user_config.axisActions = vector<AxisAction>(axisCount, AxisAction::NONE);
            int surge = 0;
            int sway = 0;
            int heave = 0;
            int yaw = 0;
            bool brightenLED = false;
            bool dimLED = false;
            bool turnFrontServoCw = false;
            bool turnFrontServoCcw = false;
            bool turnBackServoCw = false;
            bool turnBackServoCcw = false;
            // ########################
            // Add more buttons
            // ########################
            for (int i = 0; i < buttonCount; i++) {
                if (!buttons[i]) continue;
                switch (user_config.buttonActions[i]) { 
                    case ButtonAction::SURGE_BACKWARD:
                        surge -= 100;
                        break;
                    case ButtonAction::SURGE_FORWARD:
                        surge += 100;
                        break;
                    case ButtonAction::SWAY_LEFT:
                        sway -= 100;
                        break;
                    case ButtonAction::SWAY_RIGHT:
                        sway += 100;
                        break;
                    case ButtonAction::HEAVE_DOWN:
                        heave -= 100;
                        break;
                    case ButtonAction::HEAVE_UP:
                        heave += 100;
                        break;
                    case ButtonAction::YAW_LEFT:
                        yaw -= 100;
                        break;
                    case ButtonAction::YAW_RIGHT:
                        yaw += 100;
                        break;
                    case ButtonAction::BRIGHTEN_LED:
                        brightenLED = true;
                        break;
                    case ButtonAction::DIM_LED:
                        dimLED = true;
                        break;
                    case ButtonAction::TURN_FRONT_SERVO_CW:
                        turnFrontServoCw = true;
                        break;
                    case ButtonAction::TURN_FRONT_SERVO_CCW:
                        turnFrontServoCcw = true;
                        break;
                    case ButtonAction::TURN_BACK_SERVO_CW:
                        turnBackServoCw = true;
                        break;
                    case ButtonAction::TURN_BACK_SERVO_CCW:
                        turnBackServoCcw = true;
                        break;
                    case ButtonAction::CONFIGURATION_MODE:
                        // Can either be set by user input for through the GUI
                        configuration_mode = !configuration_mode;
                        break;
                    default:
                        break;
                }
            }
            for (size_t i = 0; i < user_config.axisActions.size(); i++) {
                if (std::abs(axes[i]) <= user_config.deadzone) continue;
                switch (user_config.axisActions[i]) {
                    case AxisAction::SURGE:
                        if (surge == 0) surge = -(int)(axes[i]*100);
                        break;
                    case AxisAction::SWAY:
                        if (sway == 0) sway = -(int)(axes[i]*100);
                        break;
                    case AxisAction::YAW:
                        if (yaw == 0) yaw = -(int)(axes[i]*100);
                        break;
                    case AxisAction::HEAVE:
                        if (heave == 0) heave = -(int)(axes[i]*100);
                        break;
                    default:
                        break;
                }
            }

            //should probably move this out of the main render loop?
            // ########################
            // Add more inputs here
            // ########################
            pilotInputNode->sendInput(power, surge, sway, heave, yaw, brightenLED, dimLED, turnFrontServoCw,
            turnFrontServoCcw, turnBackServoCw, turnBackServoCcw, configurationMode, configuration_mode_thruster_number);
        }

        //top menu bar
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("Cameras")) {
                if (ImGui::MenuItem("Open Camera Window")) {
                    showCameraWindow = true;
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("UserConfig")) {
                if (ImGui::MenuItem("Open UserConfig Editor")) {
                    showConfigWindow = true;
                }
                if (ImGui::BeginMenu("Load UserConfig")) {
                    for (size_t i = 0; i < names.size(); i++) {
                        if (names[i] == "waterwitch_config")
                        {
                            json configData = json::parse(configs[i]);
                            if (!configData["servos"][0].is_null()) std::strncpy(waterwitch_config.servo1ip, configData["servos"][0].get<std::string>().c_str(), sizeof(waterwitch_config.servo1ip));
                            if (!configData["servos"][1].is_null()) std::strncpy(waterwitch_config.servo2ip, configData["servos"][1].get<std::string>().c_str(), sizeof(waterwitch_config.servo2ip));
                            for (size_t i = 0; i < std::size(waterwitch_config.thruster_map); i++){
                                if (!configData["thruster_map"][1].is_null()) std::strncpy(waterwitch_config.thruster_map[i], configData["thruster_map"][i].get<std::string>().c_str(), sizeof(waterwitch_config.thruster_map[i]));
                            }
                            continue;
                        }
                        if (ImGui::MenuItem(names[i].c_str())) {
                            json configData = json::parse(configs[i]);
                            if (!configData["cameras"][0].is_null()) std::strncpy(
                                user_config.cam1ip, configData["cameras"][0].get<std::string>().c_str(), sizeof(user_config.cam1ip));
                            if (!configData["cameras"][1].is_null()) std::strncpy(user_config.cam2ip, configData["cameras"][1].get<std::string>().c_str(), sizeof(user_config.cam2ip));
                            if (!configData["cameras"][2].is_null()) std::strncpy(user_config.cam3ip, configData["cameras"][2].get<std::string>().c_str(), sizeof(user_config.cam3ip));
                            user_config.deadzone = configData.value("deadzone", 0.1f);
                            user_config.buttonActions.clear();
                            for (auto& mapping : configData["mappings"]["0"]["buttons"].items()) {
                                user_config.buttonActions.push_back(stringToButtonAction(mapping.value()));
                            }
                            user_config.axisActions.clear();
                            for (auto& mapping : configData["mappings"]["0"]["axes"].items()) {
                                user_config.axisActions.push_back(stringToAxisAction(mapping.value()));
                            }
                        }
                    
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();                
            }
            if (ImGui::BeginMenu("Pilot")) {
                if (ImGui::MenuItem("Open Piloting Menu")) {
                    showPilotWindow = true;
                }
                ImGui::EndMenu();
            }

            //fps counter
            ImGui::SameLine(ImGui::GetWindowWidth() - 100);
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

            ImGui::EndMainMenuBar();
        }

        //user_config window
        if (showConfigWindow) {
            ImGui::Begin("Config Editor", &showConfigWindow);
            if (ImGui::BeginTabBar("Config Tabs")) {
                if (ImGui::BeginTabItem("Cameras (User)")) {
                    ImGui::Text("Camera 1 URL");
                    ImGui::SameLine(); 
                    ImGui::InputText("##camera1", user_config.cam1ip, 64);
                    ImGui::Text("Camera 2 URL");
                    ImGui::SameLine(); 
                    ImGui::InputText("##camera2", user_config.cam2ip, 64);
                    ImGui::Text("Camera 3 URL");
                    ImGui::SameLine(); 
                    ImGui::InputText("##camera3", user_config.cam3ip, 64);
                    ImGui::EndTabItem();
                    
                }
                if (ImGui::BeginTabItem("Servo (Waterwitch)")){
                    ImGui::Text("Servo 1 IP");
                    ImGui::SameLine();
                    ImGui::InputText("##servo1", waterwitch_config.servo1ip, 64);
                    ImGui::Text("Servo 2 IP");
                    ImGui::SameLine();
                    ImGui::InputText("##servo2", waterwitch_config.servo2ip, 64);
                    ImGui::EndTabItem();

                }
                if (ImGui::BeginTabItem("Thruster Map (Waterwitch)")){
                    ImGui::Text("For Star (Forward Right)");
                    ImGui::SameLine();
                    ImGui::InputText("##for_star", waterwitch_config.thruster_map[0], 64);
                    ImGui::Text("For Port (Forward Left)");
                    ImGui::SameLine();
                    ImGui::InputText("##for_port", waterwitch_config.thruster_map[1], 64);
                    ImGui::Text("Aft star (Back Right)");
                    ImGui::SameLine();
                    ImGui::InputText("##aft_star", waterwitch_config.thruster_map[2], 64);
                    ImGui::Text("Aft Port (Back Left)");
                    ImGui::SameLine();
                    ImGui::InputText("##aft_port", waterwitch_config.thruster_map[3], 64);
                    ImGui::Text("Star Top (Right Top)");
                    ImGui::SameLine();
                    ImGui::InputText("##star_top", waterwitch_config.thruster_map[4], 64);
                    ImGui::Text("[Port Top (Left Top)");
                    ImGui::SameLine();
                    ImGui::InputText("##Port_top", waterwitch_config.thruster_map[5], 64);
                    ImGui::Text("Configuration Mode Thruster Number");
                    ImGui::SameLine();
                    const char* thrusterNumbers[] = { "0", "1", "2", "3", "4", "5" };
                    static int currentThrusterNumber = 0;
                    if (ImGui::Combo("##thruster_number", &currentThrusterNumber, thrusterNumbers, IM_ARRAYSIZE(thrusterNumbers))) {
                        configuration_mode_thruster_number = currentThrusterNumber;
                    }
                    ImGui::Checkbox("Configuration Mode", &configuration_mode);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Controls (User)")) {
                    if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
                        int buttonCount, axisCount;
                        const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttonCount);
                        const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axisCount);
                        ImGui::Text("Connected Controller: %s", glfwGetJoystickName(GLFW_JOYSTICK_1));
                        ImGui::Text("Axis Deadzone");
                        ImGui::SameLine();
                        ImGui::SliderFloat("##deadzone", &user_config.deadzone, 0.f, 1.f);
                        ImGui::SeparatorText("Buttons");
                        for (int i = 0; i < buttonCount; i++) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, !buttons[i], 1, 1));
                            ImGui::Text("%s", (std::string("Button ") + std::to_string(i)).c_str());
                            ImGui::PopStyleColor();
                            ImGui::SameLine(); 
                            if (ImGui::Combo((std::string("##button") + std::to_string(i)).c_str(), reinterpret_cast<int*>(&user_config.buttonActions[i]), buttonActionLabels, static_cast<int>(ButtonAction::SIZE))) {

                            }
                        }
                        ImGui::SeparatorText("Axes");
                        for (int i = 0; i < axisCount; i++) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1-abs(axes[i]), 1, 1));
                            ImGui::Text("%s", (std::string("Axis ") + std::to_string(i)).c_str());
                            ImGui::PopStyleColor();
                            ImGui::SameLine();
                            if (ImGui::Combo((std::string("##axis") + std::to_string(i)).c_str(), reinterpret_cast<int*>(&user_config.axisActions[i]), axisActionLabels, static_cast<int>(AxisAction::SIZE))) {
                                
                            }
                        }
                    } else {
                        ImGui::Text("No Controller Connected.");
                    }
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Save")) {
                    if (ImGui::Button("Save Global Config")) {
                        json configJson;
                        configJson["name"] = "waterwitch_config";
                        configJson["servos"][0] = waterwitch_config.servo1ip;
                        configJson["servos"][1] = waterwitch_config.servo2ip;

                        for (size_t i = 0; i < waterwitch_config.thruster_map.size(); i++){
                            configJson["thruster_map"][i] = waterwitch_config.thruster_map[i];
                        }
                        
                        saveConfigNode->saveConfig("waterwitch_config", configJson.dump());
                    }
                    ImGui::Text("User Config Name");
                    ImGui::SameLine(); 
                    ImGui::InputText("##configName", user_config.name, 64);
                    if (!glfwJoystickPresent(GLFW_JOYSTICK_1)) {
                        ImGui::Text("Controller must be connected");
                    } else if (ImGui::Button("Save")) {
                        json configJson;
                        configJson["name"] = user_config.name;
                        configJson["cameras"][0] = user_config.cam1ip;
                        configJson["cameras"][1] = user_config.cam2ip;
                        configJson["cameras"][2] = user_config.cam3ip;
                        configJson["controller1"] = glfwGetJoystickName(GLFW_JOYSTICK_1);
                        configJson["controller2"] = "null";
                        configJson["deadzone"] = user_config.deadzone;
                        for (size_t i = 0; i < user_config.axisActions.size(); i++) {
                            configJson["mappings"]["0"]["deadzones"][std::to_string(i)] = user_config.deadzone; //for compatability with react gui
                        }
                        for (size_t i = 0; i < user_config.buttonActions.size(); i++) {
                            configJson["mappings"]["0"]["buttons"][i] = buttonActionCodes[static_cast<int>(user_config.buttonActions[i])];
                        }
                        for (size_t i = 0; i < user_config.axisActions.size(); i++) {
                            configJson["mappings"]["0"]["axes"][i] = axisActionCodes[static_cast<int>(user_config.axisActions[i])];
                        }
                        saveConfigNode->saveConfig(string(user_config.name), configJson.dump());
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
            ImGui::Begin("Camera Window", &showCameraWindow, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        
            ImVec2 windowPos = ImGui::GetWindowPos();
            ImVec2 availPos = ImGui::GetContentRegionAvail();

            ImGui::SetCursorPos(ImVec2(windowPos.x+8, windowPos.y+8));
            cam1.render(ImVec2((availPos.x-24)/2, (availPos.y-24)/2));
            ImGui::SetCursorPos(ImVec2((availPos.x-24)/2+16, windowPos.y+8));
            cam2.render(ImVec2((availPos.x-24)/2, (availPos.y-24)/2));
            ImGui::SetCursorPos(ImVec2(windowPos.x+8, (availPos.y-24)/2+35));
            cam3.render(ImVec2((availPos.x-24)/2, (availPos.y-24)/2));
            ImGui::SetCursorPos(ImVec2((availPos.x-24)/2+16, (availPos.y-24)/2+35));
            ImGui::Image((ImTextureID)(intptr_t)eerLogo, ImVec2((availPos.x-24)/2, (availPos.y-24)/2));

            ImGui::End();
        }

        //co-pilot controls window
        if (showPilotWindow) {
            ImGui::Begin("Co-Pilot Window", &showPilotWindow);
            
            ImGui::SliderInt("Power", &power.power, 0, 100);
            ImGui::SliderInt("Surge", &power.surge, 0, 100);
            ImGui::SliderInt("Sway", &power.sway, 0, 100);
            ImGui::SliderInt("Heave", &power.heave, 0, 100);
            ImGui::SliderInt("Turn", &power.roll, 0, 100);
            ImGui::SliderInt("Turn", &power.yaw, 0, 100);

            ImGui::SeparatorText("Keybinds");
            ImGui::Text("1 - Set all to 0%%");
            ImGui::Text("2 - Set all to 50%%");
            ImGui::Text("3 - Set all to 0%%, set Heave and Power to 100%%");

            if (ImGui::IsKeyPressed(ImGuiKey_1)) {
                power.power = 0;
                power.surge = 0;
                power.sway = 0;
                power.heave = 0;
                power.roll = 0;
                power.yaw = 0;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_2)) {
                power.power = 50;
                power.surge = 50;
                power.sway = 50;
                power.heave = 50;
                power.roll = 50;
                power.yaw = 50;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_3)) {
                power.power = 100;
                power.surge = 0;
                power.sway = 0;
                power.heave = 100;
                power.roll = 0;
                power.yaw = 0;
            }

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

    rclcpp::shutdown();
    return 0;
}