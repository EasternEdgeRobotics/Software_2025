#include <thread>
#include <atomic>
#include <iostream>
#include <string>
#include <mutex>
#include <cstdio> // For freopen
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <chrono>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "eer_messages/msg/pilot_input.hpp"
#include "controller.h"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void publishControllerInput(const eer_messages::msg::PilotInput &input, rclcpp::Publisher<eer_messages::msg::PilotInput>::SharedPtr publisher) {
    publisher->publish(input);
}

//this is the fuction that will launch the GUI
void launchController(){

    // Initialize ROS 2
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("controller_input_publisher");
    auto publisher = node->create_publisher<eer_messages::msg::PilotInput>("/PilotInput", 10);

    // Start a separate thread to spin the ROS 2 node
    std::thread ros_spin_thread([&]() {
        rclcpp::spin(node);
    });

    // Redirect stderr to /dev/null to suppress GStreamer warnings
    freopen("/dev/null", "w", stderr);

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "EER Controller Input", NULL, NULL);
    if (window == NULL)
        return;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // disable vsync

    // Initialize OpenGL loader (GLFW in this case)
    if (!glfwGetProcAddress("glGenTextures")) {
        std::cerr << "Failed to initialize OpenGL loader!" << std::endl;
        return;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    //dot position and radius
    ImVec2 dot_position = ImVec2(640, 360);
    const float dot_radius = 5.0f;

    eer_messages::msg::PilotInput input;


    const std::chrono::milliseconds loop_duration(10); // 100 Hz -> 10 ms per loop iteration

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        //if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        //{
        //    std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //    continue;
        //}

        auto loop_start_time = std::chrono::steady_clock::now();
        
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        ImGui::Begin("Controller Box");
        ImGui::Text("Use left stick on controller to move dot");

        ImVec2 box_min = ImGui::GetCursorScreenPos();
        ImVec2 box_max = ImVec2(box_min.x + 1000, box_min.y + 1000);
        ImGui::InvisibleButton("box", ImVec2(1000, 1000));
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRect(box_min, box_max, IM_COL32(255, 255, 255, 255));

        dot_position.x = std::max(box_min.x + dot_radius, std::min(dot_position.x, box_max.x - dot_radius));
        dot_position.y = std::max(box_min.y + dot_radius, std::min(dot_position.y, box_max.y - dot_radius));

        dot_position.x += input.sway / 127.0f;
        dot_position.y += input.surge / 127.0f;

        draw_list->AddCircleFilled(dot_position, dot_radius, IM_COL32(255, 0, 0, 255));

        ImGui::End();

        if (glfwJoystickPresent(GLFW_JOYSTICK_1)) {
            int axesCount, buttonsCount;
            const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axesCount);
            const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttonsCount);

            if (axesCount >= 5) { // Ensure there are enough axes
                input.surge = static_cast<int8_t>(axes[1] * -127); // Left thumbstick vertical axis
                input.sway = static_cast<int8_t>(axes[0] * 127);  // Left thumbstick horizontal axis

                input.pitch = static_cast<int8_t>(axes[4] * 127); // Right thumbstick horizontal axis
                input.yaw = static_cast<int8_t>(axes[3] * 127); // Right thumbstick horizontal axis
            }

            for (int i = 0; i < buttonsCount; i++) {
                bool button_state = (buttons[i] == GLFW_PRESS);
                switch (i) {
                    case 0: break;//x button
                    case 1: input.is_autonomous = button_state;break;
                    case 2: break; 
                    case 3: break; 
                    case 4: input.pitch_up = button_state; break; //left bumper
                    case 5: input.pitch_down = button_state; break; //right bumper
                    case 6: input.open_claw = button_state; break; // left trigger
                    case 7: input.close_claw = button_state; break; //right trigger
                    case 8: break;
                    case 9: break;
                    case 10: break;
                    case 11: break;
                    case 12: break;
                    case 13: input.heave_up = button_state; break; //Dpad up
                    case 14: break;
                    case 15: input.heave_down = button_state; break;//Dpad down
                    case 16: break;
                    default: break;
                }
            }

            ImGui::Begin("Controller Input");
            ImGui::Text("Axes:");
            ImGui::Text("Surge: %d", input.surge);
            ImGui::Text("Sway: %d", input.sway);
            ImGui::Text("Heave: %d", input.heave);
            ImGui::Text("Pitch: %d", input.pitch);
            ImGui::Text("Yaw: %d", input.yaw);

            ImGui::Text("Buttons:");
            ImGui::Text("Is autonomous: %s", input.is_autonomous ? "Pressed" : "Released");
            ImGui::Text("Open Claw: %s", input.open_claw ? "Pressed" : "Released");
            ImGui::Text("Close Claw: %s", input.close_claw ? "Pressed" : "Released");
            ImGui::Text("Heave Up: %s", input.heave_up ? "Pressed" : "Released");
            ImGui::Text("Heave Down: %s", input.heave_down ? "Pressed" : "Released");
            ImGui::Text("Pitch Up: %s", input.pitch_up ? "Pressed" : "Released");
            ImGui::Text("Pitch Down: %s", input.pitch_down ? "Pressed" : "Released");
            ImGui::Text("Brighten LED: %s", input.brighten_led ? "Pressed" : "Released");
            ImGui::End();
        }

        publishControllerInput(input, publisher);


        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        // Sleep to maintain 100 Hz loop rate
        auto loop_end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
        if (elapsed_time < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed_time);
        }
    }


    rclcpp::shutdown();
    ros_spin_thread.join();

#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}