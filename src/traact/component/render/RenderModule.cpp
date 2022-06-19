/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "RenderModule.h"
#include "RenderModuleComponent.h"

namespace traact::component::render {

static void glfw_error_callback(int error, const char *description) {
    SPDLOG_ERROR("Glfw Error {0}: {1}\n", error, description);
}

bool RenderModule::init(Module::ComponentPtr module_component) {
    {
        std::scoped_lock lock(data_lock_);
        if (running_)
            return true;
        SPDLOG_DEBUG("Starting Render Module");
        running_ = true;
    }

    auto update_rate_float = TimeDuration(std::chrono::seconds(1)).count() / render_target_fps_;
    target_loop_time_ = TimeDuration(static_cast<int64_t>(update_rate_float));
    SPDLOG_INFO("render target fps: {0}, delta ns: {1}", render_target_fps_, target_loop_time_.count());

    std::shared_future<void> init_future(initialized_promise_.get_future());
    thread_ = std::thread([this] {
        threadLoop();
    });

    while (init_future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
        if (running_) {
            SPDLOG_INFO("RenderModule: waiting for render thread to initialize");
        }
    }

    SPDLOG_INFO("RenderModule: finished configure");
    return true;
}

bool RenderModule::start(Module::ComponentPtr module_component) {

    SPDLOG_INFO("RenderModule: finished starting");

    return true;
}

bool RenderModule::stop(Module::ComponentPtr module_component) {
    if (!running_)
        return true;
    running_ = false;

    try {
        if (thread_.joinable()) {
            thread_.join();
        }
    } catch(std::exception &e) {
        SPDLOG_ERROR(e.what());
    }

    return true;
}

bool RenderModule::teardown(Module::ComponentPtr module_component) {
    return Module::teardown(module_component);
}

size_t
RenderModule::addComponent(std::string window_name,
                           const std::string &component_name,
                           RenderComponent *component,
                           RenderConfiguration render_config) {

    std::scoped_lock lock(data_lock_);

    render_target_fps_ = render_config.target_fps;
    SPDLOG_INFO("set target fps to {0}", render_target_fps_);

    window_components_[window_name].push_back(component);
    all_render_component_names_.push_back(component_name);
    render_size_[window_name] = {};
    image_size_[window_name] = {};
    camera_calibration_[window_name] = {};

}

void RenderModule::threadLoop() {

    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit()) {
        SPDLOG_ERROR("could not initialize glfw");
        return;
    }

#if __APPLE__
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);		   // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    GLFWwindow *window = glfwCreateWindow(800, 600, "glfw window", nullptr, nullptr);

    if (window == NULL) {
        SPDLOG_ERROR("could not create glfw window");
        return;
    }

    //glfwSetWindowCloseCallback( window, []( GLFWwindow* window ){ glfwSetWindowShouldClose( window, GL_FALSE ); } );
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);// Enable vsync


    if (glewInit() != GLEW_OK) {
        SPDLOG_ERROR("Failed to initialize OpenGL loader!");
        return;
    }

    int screen_width, screen_height;
    glfwGetFramebufferSize(window, &screen_width, &screen_height);
    glViewport(0, 0, screen_width, screen_height);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImGui::StyleColorsDark();

    SPDLOG_DEBUG("call render component renderInit");
    glfwPollEvents();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    for (const auto &window_component : window_components_) {
        std::string window_name = window_component.first;
        for (auto &component : window_component.second) {
            component->renderInit();
        }
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    bool window_open = true;

    initialized_promise_.set_value();

    fps_render_.event();
    for (const auto &window : window_components_) {
        fps_new_data_[window.first].event();
    }
    SPDLOG_DEBUG("start render loop");
    while (running_) {

        auto render_start = nowSteady();

        glfwPollEvents();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        bool additional_commands_processed{false};
        {
            std::scoped_lock lock(data_lock_);
            //SPDLOG_INFO("render new frame");
            for (auto &window_commands : current_render_commands_) {
                std::string window_name = window_commands.first;

                auto &all_additional_commands = current_additional_commands_[window_name];
                auto &all_commands = window_commands.second;
                if (all_commands.empty()) {
                    SPDLOG_ERROR("no render events for windows {0}", window_name);
                    continue;
                }

                ImGui::Begin(window_name.c_str());

                if (!all_additional_commands.empty()) {
                    for (auto &command : all_additional_commands) {
                        SPDLOG_TRACE("additional command ts: {0} component: {1}",
                                     command->GetMeaIdx(),
                                     command->GetComponentName());
                        command->DoRender();
                    }
                    all_additional_commands.clear();
                    //fps_new_data_[window_name].event();
                    additional_commands_processed = true;
                }

                for (auto &command : all_commands) {
                    //SPDLOG_INFO("render command ts: {0} component: {1}", command->GetMeaIdx(), command->GetComponentName());
                    command->DoRender();
                }
                ImGui::End();
            }
        }
        if (additional_commands_processed) {
            SPDLOG_TRACE("render command ts: additional_commands_processed_");
            additional_commands_processed_.SetInit(true);
        }

        fps_render_.event();
        ImGui::Begin("Renderer Stats");
        ImGui::Text("Frontend fps: %f", fps_render_.fps());
        for (const auto &fps_counter : fps_new_data_) {
            ImGui::Text("%s fps: %f", fps_counter.first.c_str(), fps_counter.second.fps());
        }
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        window_open = !glfwWindowShouldClose(window);



        // limit fps
        auto render_end = nowSteady();
        auto render_time = render_end - render_start;

        auto wait_time = target_loop_time_ - render_time;
        //SPDLOG_TRACE("render sleep render time {0} wait time {1}",
        //                   std::chrono::duration_cast<std::chrono::milliseconds>(render_time).count(),
        //                   std::chrono::duration_cast<std::chrono::milliseconds>(wait_time).count());
        if (wait_time > std::chrono::milliseconds(1)) {
            std::this_thread::sleep_for(wait_time);
            auto after_sleep = nowSteady();
            //SPDLOG_TRACE("render sleep total {0}", (after_sleep - render_end).count());
        }

        if (!window_open) {
            //SPDLOG_TRACE("close window");
            // at least one window with one component must exist
            // get first component of first window to call finished
            window_components_.begin()->second.front()->setSourceFinished();
            //SPDLOG_TRACE("close window finished");
        }
        //SPDLOG_DEBUG("render loop finished");
    }

    glfwPollEvents();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    for (const auto &window_component : window_components_) {
        std::string window_name = window_component.first;
        for (auto &component : window_component.second) {
            component->renderStop();
        }
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    ImGui_ImplGlfw_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}
void RenderModule::updateCurrentRenderCommands() {
    std::scoped_lock lock(data_lock_);
    SPDLOG_TRACE("RenderModule: update current render events");
    bool all_windows_valid = true;
    for (auto &window_components : window_components_) {
        std::string window_name = window_components.first;

        auto &all_commands = render_commands_[window_name];

        if (all_commands.empty()) {
            SPDLOG_TRACE("no render commands for window {0}, ok for non data messages", window_name);
            all_windows_valid = false;
            break;
        }
        if (all_commands.size() != window_components_[window_name].size()) {
            all_windows_valid = false;
            break;
        }
    }

    if (!all_windows_valid) {
        return;
    }

    for (auto &window_components : window_components_) {
        std::string window_name = window_components.first;
        fps_new_data_[window_name].event();

        auto &all_commands = render_commands_[window_name];

        std::sort(all_commands.begin(), all_commands.end(),
                  [](const RenderCommand::Ptr &a, const RenderCommand::Ptr &b) -> bool {
                      return a->GetPriority() < b->GetPriority();
                  });

        current_render_commands_[window_name].swap(all_commands);
        all_commands.clear();
        SPDLOG_TRACE("added commands, current: {0}, next: {1}",
                     current_render_commands_[window_name].size(),
                     all_commands.size());

        auto &all_add_commands = additional_commands_[window_name];
        if (!all_add_commands.empty()) {
            std::sort(all_add_commands.begin(), all_add_commands.end(),
                      [](const RenderCommand::Ptr &a, const RenderCommand::Ptr &b) -> bool {
                          return a->GetPriority() < b->GetPriority();
                      });
            current_additional_commands_[window_name].swap(all_add_commands);
            SPDLOG_TRACE("added additional commands, current: {0}, next: {1}",
                         current_additional_commands_[window_name].size(),
                         all_add_commands.size());
            additional_commands_processed_.SetInit(false);
        }
    }

}

RenderModule::RenderModule() : thread_() {
    additional_commands_processed_.SetInit(true);
}

void RenderModule::setComponentReady(RenderCommand::Ptr render_command) {
    if (render_command->GetMeaIdx() == 0) {
        SPDLOG_ERROR("Timestamp is 0: {0} {1}", render_command->GetWindowName(), render_command->GetComponentName());
        return;
    }

    {
        std::scoped_lock lock(data_lock_);
        render_commands_[render_command->GetWindowName()].push_back(render_command);
    }
}
void RenderModule::setImageRenderSize(ImVec2 render_size, const std::string &window) {
    render_size_[window] = render_size;
}

const std::optional<ImVec2> &RenderModule::getImageRenderSize(const std::string &window) const {
    return render_size_.at(window);
}

void RenderModule::addAdditionalCommand(RenderCommand::Ptr render_command) {
    {
        std::scoped_lock lock(data_lock_);
        additional_commands_[render_command->GetWindowName()].push_back(render_command);
    }
    additional_commands_processed_.SetInit(false);

}
void RenderModule::processTimePoint() {
    SPDLOG_TRACE("update current render commands: start");
    updateCurrentRenderCommands();
    SPDLOG_TRACE("update current render commands: start waiting");
    TimestampSteady start = nowSteady();
    while (!additional_commands_processed_.tryWait() && running_) {
        SPDLOG_WARN(
            "timeout waiting for additional render commands of render module to be processed (e.g. image texture upload)");
    }
    auto end = nowSteady();
    SPDLOG_DEBUG("update current render commands: done in {0}", end - start);
}
void RenderModule::setCameraCalibration(const vision::CameraCalibration &camera_calibration,
                                        const std::string &window) {
    camera_calibration_[window] = camera_calibration;
}
const std::optional<vision::CameraCalibration> &RenderModule::getCameraCalibration(const std::string &window) const {
    return camera_calibration_.at(window);
}
void RenderModule::setImageSize(ImVec2 image_size, const std::string &window) {
    image_size_[window] = image_size;
}
const std::optional<ImVec2> &RenderModule::getImageSize(const std::string &window) const {
    return image_size_.at(window);
}

void RenderCommand::DoRender() {
    if (callback_)
        callback_();
}

RenderCommand::RenderCommand(std::string window_name, std::string component_name,
                             size_t mea_idx, size_t priority,
                             RenderCommand::CalledFromRenderer callback)
    :
    window_name_(std::move(window_name)),
    component_name_(std::move(component_name)),
    mea_idx_(mea_idx),
    priority_(priority),
    callback_(std::move(callback)) {

}

const std::string &RenderCommand::GetComponentName() const {
    return component_name_;
}

const std::string &RenderCommand::GetWindowName() const {
    return window_name_;
}

size_t RenderCommand::GetMeaIdx() const {
    return mea_idx_;
}

size_t RenderCommand::GetPriority() const {
    return priority_;
}

RenderCommand::RenderCommand(std::string window_name, std::string component_name,
                             size_t mea_idx, size_t priority) :
    window_name_(std::move(window_name)),
    component_name_(std::move(component_name)),
    mea_idx_(mea_idx),
    priority_(priority) {

}
void RenderCommand::updateMeaIdxForReuse(size_t new_mea_idx) {
    mea_idx_ = new_mea_idx;

}

}