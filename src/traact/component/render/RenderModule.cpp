/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "RenderModule.h"

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

static void glfw_error_callback(int error, const char *description) {
    SPDLOG_ERROR("Glfw Error {0}: {1}\n", error, description);
}

bool traact::component::render::RenderModule::init(traact::component::Module::ComponentPtr module_component) {
    {
        std::scoped_lock lock(data_lock_);
        if (running_)
            return true;
        SPDLOG_DEBUG("Starting Render Module");
        running_ = true;
    }

    std::shared_future<void> init_future(initialized_promise_.get_future());
    thread_ = std::thread([this] {
        thread_loop();
    });

    while (init_future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
        if (running_) {
            SPDLOG_INFO("RenderModule: waiting for render thread to initialize");
        }
    }

    SPDLOG_INFO("RenderModule: finished configure");
    return true;
}

bool traact::component::render::RenderModule::start(traact::component::Module::ComponentPtr module_component) {

    SPDLOG_INFO("RenderModule: finished starting");

    return true;
}

bool traact::component::render::RenderModule::stop(traact::component::Module::ComponentPtr module_component) {
    if (!running_)
        return true;
    running_ = false;
    thread_.join();
    return true;
}

bool traact::component::render::RenderModule::teardown(traact::component::Module::ComponentPtr module_component) {
    return Module::teardown(module_component);
}

size_t
traact::component::render::RenderModule::addComponent(std::string window_name, const std::string &component_name,
                                                      RenderComponent *component,
                                                      size_t priority) {

    std::scoped_lock lock(data_lock_);

    window_components_[window_name].push_back(component);
    all_render_component_names_.push_back(component_name);

}

void traact::component::render::RenderModule::thread_loop() {

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

    glfwPollEvents();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    for (const auto &window_component : window_components_) {
        std::string window_name = window_component.first;

        //ImGui::Begin( window_component.first.c_str(), &running_ );

        for (auto &component : window_component.second) {
            component->RenderInit();
        }

        //ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    bool window_open = true;

    initialized_promise_.set_value();

    while (running_ && window_open) {

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //Timestamp ts;
        bool ready = true;

        {
            std::scoped_lock lock(data_lock_);

            for (auto &window_components : window_components_) {
                std::string window_name = window_components.first;

                auto &all_commands = render_commands_[window_name];

                if (all_commands.empty()) {
                    continue;
                }

                auto &ts_commands = *all_commands.begin();
                size_t mea_idx = ts_commands.first;
                auto &commands = ts_commands.second;
                bool window_ready = commands.size() == window_components_[window_name].size();

                if (!window_ready)
                    continue;

                std::sort(commands.begin(), commands.end(),
                          [](const RenderCommand::Ptr &a, const RenderCommand::Ptr &b) -> bool {
                              return a->GetPriority() < b->GetPriority();
                          });

                current_render_commands_[window_name] = commands;
                all_commands.erase(mea_idx);
            }
        }

        if (ready) {
            glfwPollEvents();
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            for (auto &window_commands : current_render_commands_) {
                std::string window_name = window_commands.first;
                auto &all_commands = window_commands.second;

                if (all_commands.empty()) {
                    SPDLOG_ERROR("no render events for windows {0}", window_name);
                    continue;
                }

                ImGui::Begin(window_name.c_str());

                for (auto &command : all_commands) {
                    command->DoRender();
                }

                ImGui::End();


                //render_ready_[window_name].erase(window_ready_data);
            }

            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
        }

        window_open = !glfwWindowShouldClose(window);
    }

    ImGui_ImplGlfw_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

traact::component::render::RenderModule::RenderModule() : thread_() {

}

void traact::component::render::RenderModule::setComponentReady(RenderCommand::Ptr render_command) {
    if (render_command->GetMeaIdx() == 0) {
        SPDLOG_ERROR("Timestamp is 0: {0} {1}", render_command->GetWindowName(), render_command->GetComponentName());
        return;
    }

    std::scoped_lock lock(data_lock_);
    render_commands_[render_command->GetWindowName()][render_command->GetMeaIdx()].push_back(render_command);

}
void traact::component::render::RenderModule::setImageRenderSize(ImVec2 render_size) {
    render_size_ = render_size;
}
std::optional<ImVec2> traact::component::render::RenderModule::getImageRenderSize() {
  return render_size_;
}

traact::component::render::RenderComponent::RenderComponent(const std::string &name) : ModuleComponent(name,
                                                                                                       ComponentType::SYNC_SINK,
                                                                                                       ModuleType::GLOBAL) {

}

std::string traact::component::render::RenderComponent::getModuleKey() {
    return "GlobalRenderModule";
}

traact::component::Module::Ptr traact::component::render::RenderComponent::instantiateModule() {
    return std::make_shared<RenderModule>();
}

bool traact::component::render::RenderComponent::configure(const nlohmann::json &parameter,
                                                           traact::buffer::ComponentBufferConfig *data) {
    render_module_ = std::dynamic_pointer_cast<RenderModule>(module_);
    pattern::setValueFromParameter(parameter, "window", window_name_, "invalid");
    pattern::setValueFromParameter(parameter, "priority", priority_, 1);
    render_module_->addComponent(window_name_, getName(), this, priority_);
    return ModuleComponent::configure(parameter, data);
}

void traact::component::render::RenderComponent::invalidTimePoint(traact::Timestamp ts, size_t mea_idx) {
    //if(ts == Timestamp::min())
    //    return;
    auto command = std::make_shared<RenderCommand>(window_name_, getName(), ts.time_since_epoch().count(), priority_);
    render_module_->setComponentReady(command);
    //releaseAsyncCall(ts);
}

void traact::component::render::RenderCommand::DoRender() {
    if (callback_)
        callback_();
}

traact::component::render::RenderCommand::RenderCommand(std::string window_name, std::string component_name,
                                                        size_t mea_idx, size_t priority,
                                                        traact::component::render::RenderCommand::CalledFromRenderer callback)
    :
    window_name_(std::move(window_name)),
    component_name_(std::move(component_name)),
    mea_idx_(mea_idx),
    priority_(priority),
    callback_(std::move(callback)) {

}

const std::string &traact::component::render::RenderCommand::GetComponentName() const {
    return component_name_;
}

const std::string &traact::component::render::RenderCommand::GetWindowName() const {
    return window_name_;
}

size_t traact::component::render::RenderCommand::GetMeaIdx() const {
    return mea_idx_;
}

size_t traact::component::render::RenderCommand::GetPriority() const {
    return priority_;
}

traact::component::render::RenderCommand::RenderCommand(std::string window_name, std::string component_name,
                                                        size_t mea_idx, size_t priority) :
    window_name_(std::move(window_name)),
    component_name_(std::move(component_name)),
    mea_idx_(mea_idx),
    priority_(priority) {

}

