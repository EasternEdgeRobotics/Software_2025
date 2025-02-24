#include <GLFW/glfw3.h>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "rclcpp/rclcpp.hpp"
#include "eer_interfaces/srv/list_config.hpp"
#include "eer_interfaces/msg/save_config.hpp"
#include "eer_interfaces/msg/pilot_input.hpp"
#include "eer_interfaces/msg/thruster_multipliers.hpp"
#include <nlohmann/json.hpp>
#include "Config.hpp"
#include "Power.hpp"
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

using json = nlohmann::json;

Config config;
Power power;

bool showConfigWindow = false;
bool showCameraWindow = false;
bool showPilotWindow = false;

vector<string> names;
vector<string> configs;

std::array<std::vector<std::string>, 2> getConfigs() {
    auto node = rclcpp::Node::make_shared("list_configs_client");
    auto client = node->create_client<eer_interfaces::srv::ListConfig>("list_configs");
    auto request = std::make_shared<eer_interfaces::srv::ListConfig::Request>();
  
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return std::array<std::vector<std::string>, 2>();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    std::array<std::vector<std::string>, 2> output;
  
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        output[0] = response->names;
        output[1] = response->configs;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service list_configs");
    }

    return output;
}

class SaveConfigPublisher : public rclcpp::Node {
    public:
        SaveConfigPublisher() : Node("save_config_publisher") {
            publisher_ = this->create_publisher<eer_interfaces::msg::SaveConfig>("/save_config", 10);
        }

        void saveConfig(const std::string & name, const std::string & data) {
            auto msg = eer_interfaces::msg::SaveConfig();
            msg.name = name;
            msg.data = data;
            publisher_->publish(msg);
        }

    private:
        rclcpp::Publisher<eer_interfaces::msg::SaveConfig>::SharedPtr publisher_;
};

class PilotInputPublisher : public rclcpp::Node {
    public:
        PilotInputPublisher() : Node("pilot_input_publisher") {
            publisher_ = this->create_publisher<eer_interfaces::msg::PilotInput>("/pilot_input", 10);
        }

        void sendInput(const int& surge,
        const int& sway,
        const int& heave,
        const int& yaw,
        const bool& brightenLED,
        const bool& dimLED) {
            auto msg = eer_interfaces::msg::PilotInput();
            msg.surge = surge;
            msg.sway = sway;
            msg.heave = heave;
            msg.yaw = yaw;
            //may need to remove these? not sure if leds are on the bot this year
            msg.brighten_led = brightenLED;
            msg.dim_led = dimLED;
            publisher_->publish(msg);
        }

    private:
        rclcpp::Publisher<eer_interfaces::msg::PilotInput>::SharedPtr publisher_;
};

class ThrusterMultipliersPublisher : public rclcpp::Node {
    public:
        ThrusterMultipliersPublisher() : Node("thruster_multipliers_publisher") {
            publisher_ = this->create_publisher<eer_interfaces::msg::ThrusterMultipliers>("/thruster_multipliers", 10);
        }

        void sendPower() {
            //surge sway heave itch roll yaw
            auto msg = eer_interfaces::msg::ThrusterMultipliers();
            msg.power = power.power;
            msg.surge = power.surge;
            msg.sway = power.sway;
            msg.heave = power.heave;
            msg.pitch = 0; //pitch is not possible with our thruster configuration
            //msg.roll = 0; //roll is possible, but not used? can be discussed later
            msg.yaw = power.yaw;
            publisher_->publish(msg);
        }

    private:
        rclcpp::Publisher<eer_interfaces::msg::ThrusterMultipliers>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
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

    rclcpp::init(argc, argv);
    
    auto saveConfigNode = std::make_shared<SaveConfigPublisher>();
    auto pilotInputNode = std::make_shared<PilotInputPublisher>();
    auto thrusterMultipliersNode = std::make_shared<ThrusterMultipliersPublisher>();

    std::array<std::vector<std::string>, 2> configRes = getConfigs();
    names = configRes[0];
    configs = configRes[1];

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
            if (buttonCount > 0 && config.buttonActions.empty()) config.buttonActions = vector<ButtonAction>(buttonCount, ButtonAction::NONE);
            if (axisCount > 0 && config.axisActions.empty()) config.axisActions = vector<AxisAction>(axisCount, AxisAction::NONE);
            int surge = 0;
            int sway = 0;
            int heave = 0;
            int yaw = 0;
            bool brightenLED = false;
            bool dimLED = false;
            for (int i = 0; i < buttonCount; i++) {
                if (!buttons[i]) continue;
                switch (config.buttonActions[i]) { 
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
                    default:
                        break;
                }
            }
            for (size_t i = 0; i < config.axisActions.size(); i++) {
                if (std::abs(axes[i]) <= config.deadzone) continue;
                switch (config.axisActions[i]) {
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
            pilotInputNode->sendInput(surge, sway, heave, yaw, brightenLED, dimLED);
        }

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
                    for (size_t i = 0; i < names.size(); i++) {
                        if (ImGui::MenuItem(names[i].c_str())) {
                            json configData = json::parse(configs[i]);
                            if (!configData["cameras"][0].is_null()) std::strncpy(config.cam1ip, configData["cameras"][0].get<std::string>().c_str(), sizeof(config.cam1ip));
                            if (!configData["cameras"][1].is_null()) std::strncpy(config.cam2ip, configData["cameras"][1].get<std::string>().c_str(), sizeof(config.cam2ip));
                            if (!configData["cameras"][2].is_null()) std::strncpy(config.cam3ip, configData["cameras"][2].get<std::string>().c_str(), sizeof(config.cam3ip));
                            config.deadzone = configData.value("deadzone", 0.1f);
                            config.buttonActions.clear();
                            for (auto& mapping : configData["mappings"]["0"]["buttons"].items()) {
                                config.buttonActions.push_back(stringToButtonAction(mapping.value()));
                            }
                            config.axisActions.clear();
                            for (auto& mapping : configData["mappings"]["0"]["axes"].items()) {
                                config.axisActions.push_back(stringToAxisAction(mapping.value()));
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
                        ImGui::Text("Connected Controller: %s", glfwGetJoystickName(GLFW_JOYSTICK_1));
                        ImGui::Text("Axis Deadzone");
                        ImGui::SameLine();
                        ImGui::SliderFloat("##deadzone", &config.deadzone, 0.f, 1.f);
                        ImGui::SeparatorText("Buttons");
                        for (int i = 0; i < buttonCount; i++) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, !buttons[i], 1, 1));
                            ImGui::Text("%s", (std::string("Button ") + std::to_string(i)).c_str());
                            ImGui::PopStyleColor();
                            ImGui::SameLine(); 
                            if (ImGui::Combo((std::string("##button") + std::to_string(i)).c_str(), reinterpret_cast<int*>(&config.buttonActions[i]), buttonActionLabels, static_cast<int>(ButtonAction::SIZE))) {

                            }
                        }
                        ImGui::SeparatorText("Axes");
                        for (int i = 0; i < axisCount; i++) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1-abs(axes[i]), 1, 1));
                            ImGui::Text("%s", (std::string("Axis ") + std::to_string(i)).c_str());
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
                        configJson["cameras"][0] = config.cam1ip;
                        configJson["cameras"][1] = config.cam2ip;
                        configJson["cameras"][2] = config.cam3ip;
                        configJson["controller1"] = glfwGetJoystickName(GLFW_JOYSTICK_1);
                        configJson["controller2"] = "null";
                        configJson["deadzone"] = config.deadzone;
                        for (size_t i = 0; i < config.axisActions.size(); i++) {
                            configJson["mappings"]["0"]["deadzones"][std::to_string(i)] = config.deadzone; //for compatability with react gui
                        }
                        for (size_t i = 0; i < config.buttonActions.size(); i++) {
                            configJson["mappings"]["0"]["buttons"][i] = buttonActionCodes[static_cast<int>(config.buttonActions[i])];
                        }
                        for (size_t i = 0; i < config.axisActions.size(); i++) {
                            configJson["mappings"]["0"]["axes"][i] = axisActionCodes[static_cast<int>(config.axisActions[i])];
                        }
                        saveConfigNode->saveConfig(string(config.name), configJson.dump());
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
            ImGui::Button("Camera 1", ImVec2((availPos.x-24)/2, (availPos.y-24)/2));
            ImGui::SetCursorPos(ImVec2((availPos.x-24)/2+16, windowPos.y+8));
            ImGui::Button("Camera 2", ImVec2((availPos.x-24)/2, (availPos.y-24)/2));
            ImGui::SetCursorPos(ImVec2(windowPos.x+8, (availPos.y-24)/2+35));
            ImGui::Button("Camera 3", ImVec2((availPos.x-24)/2, (availPos.y-24)/2));

            ImGui::End();
        }

        //co-pilot controls window
        if (showPilotWindow) {
            ImGui::Begin("Co-Pilot Window", &showPilotWindow);

            if (ImGui::SliderInt("Power", &power.power, 0, 100)) {
                thrusterMultipliersNode->sendPower();
            }
            if (ImGui::SliderInt("Surge", &power.surge, 0, 100)) {
                thrusterMultipliersNode->sendPower();
            }
            if (ImGui::SliderInt("Sway", &power.sway, 0, 100)) {
                thrusterMultipliersNode->sendPower(); 
            }
            if (ImGui::SliderInt("Turn", &power.yaw, 0, 100)) {
                thrusterMultipliersNode->sendPower();
            }
            if (ImGui::SliderInt("Heave", &power.heave, 0, 100)) {
                thrusterMultipliersNode->sendPower();
            }

            ImGui::SeparatorText("Keybinds");
            ImGui::Text("1 - Set all to 0%%");
            ImGui::Text("2 - Set all to 50%%");
            ImGui::Text("3 - Set all to 0%%, set Heave and Power to 100%%");

            if (ImGui::IsKeyPressed(ImGuiKey_1)) {
                power.power = 0;
                power.surge = 0;
                power.sway = 0;
                power.yaw = 0;
                power.heave = 0;
                thrusterMultipliersNode->sendPower();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_2)) {
                power.power = 50;
                power.surge = 50;
                power.sway = 50;
                power.yaw = 50;
                power.heave = 50;
                thrusterMultipliersNode->sendPower();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_3)) {
                power.power = 100;
                power.surge = 0;
                power.sway = 0;
                power.yaw = 0;
                power.heave = 100;
                thrusterMultipliersNode->sendPower();
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