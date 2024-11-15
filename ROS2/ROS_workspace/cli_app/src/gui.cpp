 // Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp
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

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// This example can also compile and run with Emscripten! See 'Makefile.emscripten' for details.
#ifdef __EMSCRIPTEN__
#include "../libs/emscripten/emscripten_mainloop_stub.h"
#endif

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void webcamCapture(std::atomic<bool>& running, std::atomic<bool>& show_video, cv::Mat& frame, std::mutex& frame_mutex) {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open webcam" << std::endl;
        running.store(false);
        return;
    }

    // Set capture properties for better performance
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320); // Lower resolution
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // Lower resolution
    cap.set(cv::CAP_PROP_FPS, 30); // Limit frame rate

    while (running.load()) {
        if (show_video.load()) {
            cv::Mat temp_frame;
            cap >> temp_frame;
            if (!temp_frame.empty()) {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame = temp_frame.clone();
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    cap.release();
}


GLuint matToTexture(const cv::Mat &mat, GLuint textureID, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    if (textureID == 0) {
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.ptr());
    } else {
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mat.cols, mat.rows, GL_BGR, GL_UNSIGNED_BYTE, mat.ptr());
    }

    return textureID;
}




//this is the fuction that will launch the GUI
void launchGUI(){

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
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "EER GUI App", NULL, NULL);
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
    
    GLuint video_texture = 0;
    std::atomic<bool> running(true);
    std::atomic<bool> show_video(true);
    cv::Mat frame;
    std::mutex frame_mutex;

    // Initialize webcam
    //cv::VideoCapture cap(0);
    //if (!cap.isOpened()) {
    //    std::cerr << "Failed to open webcam" << std::endl;
    //    return;
    //}

    // Start webcam capture thread
    std::thread capture_thread(webcamCapture, std::ref(running), std::ref(show_video), std::ref(frame), std::ref(frame_mutex));


#ifdef __EMSCRIPTEN__
    ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
#endif


    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    // - Our Emscripten build process allows embedding fonts to be accessible at runtime from the "fonts/" folder. See Makefile.emscripten for details.
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    //dot position and radius
    ImVec2 dot_position = ImVec2(640, 360);
    const float dot_radius = 5.0f;




       eer_messages::msg::PilotInput input;

    // Main loop
    const std::chrono::milliseconds loop_duration(10); // 100 Hz -> 10 ms per loop iteration
#ifdef __EMSCRIPTEN__
    // For an Emscripten build we are disabling file-system access, so let's not attempt to do a fopen() of the imgui.ini file.
    // You may manually call LoadIniSettingsFromMemory() to load settings from your own storage.
    io.IniFilename = nullptr;
    EMSCRIPTEN_MAINLOOP_BEGIN
#else
    while (!glfwWindowShouldClose(window))
#endif
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



        // Show video frame
        ImGui::Begin("Webcam Stream");
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!frame.empty()) {
                video_texture = matToTexture(frame, video_texture, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
                ImGui::Image((ImTextureID)(intptr_t)video_texture, ImVec2(frame.cols, frame.rows));
            }
        }
        if (ImGui::Button("Close Video Stream")) {
            show_video.store(false);
        }
        if (ImGui::Button("Open Video Stream")) {
            show_video.store(true);
        }

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
                    case 0: break;
                    case 1: break;
                    case 2: input.heave_up = button_state; break;
                    case 3: input.heave_down = button_state; break;
                    case 4: input.pitch_up = button_state; break;
                    case 5: input.pitch_down = button_state; break;
                    case 6: input.open_claw = button_state; break;
                    case 7: input.close_claw = button_state; break;
                    case 8: break;
                    case 9: break;
                    case 10: break;
                    case 11: break;
                    case 12: break;
                    case 13: input.heave_up = button_state; break;
                    case 14: break;
                    case 15: input.heave_down = button_state; break;
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


        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        //if (show_demo_window)
            //ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::End();
        }

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

    // Stop the webcam capture thread
    running.store(false);
    capture_thread.join();

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