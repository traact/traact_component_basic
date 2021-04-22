/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include "RenderModule.h"

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

static void glfw_error_callback(int error, const char *description)
{
    spdlog::error( "Glfw Error {0}: {1}\n", error, description);
}

bool traact::component::render::RenderModule::init(traact::component::Module::ComponentPtr module_component) {
    return Module::init(module_component);
}

bool traact::component::render::RenderModule::start(traact::component::Module::ComponentPtr module_component) {

    if(running_)
        return true;
    SPDLOG_DEBUG("Starting Render Module");
    running_ = true;
    thread_ = std::make_shared<std::thread>([this]{
        thread_loop();
    });

    return true;
}

bool traact::component::render::RenderModule::stop(traact::component::Module::ComponentPtr module_component) {
    if(!running_)
        return true;
    running_ = false;
    thread_->join();
    return true;
}

bool traact::component::render::RenderModule::teardown(traact::component::Module::ComponentPtr module_component) {
    return Module::teardown(module_component);
}

std::size_t
traact::component::render::RenderModule::addComponent(std::string window_name, const std::string &component_name,
                                                      RenderComponent *component,
                                                      std::size_t priority) {

    std::scoped_lock lock(data_lock_);

    window_components_[window_name][priority].push_back(component);
    all_render_component_names_.push_back(component_name);


}

void traact::component::render::RenderModule::thread_loop() {

    glfwSetErrorCallback(glfw_error_callback);

    if( !glfwInit() ){
        spdlog::error("could not initialize glfw");
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


    GLFWwindow* window = glfwCreateWindow( 800, 600, "glfw window", nullptr, nullptr );
    if (window == NULL) {
        spdlog::error("could not create glfw window");
        return;
    }

    //glfwSetWindowCloseCallback( window, []( GLFWwindow* window ){ glfwSetWindowShouldClose( window, GL_FALSE ); } );
    glfwMakeContextCurrent( window );
    glfwSwapInterval( 1 );// Enable vsync


    if (glewInit() != GLEW_OK)
    {
        spdlog::error("Failed to initialize OpenGL loader!");
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
    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear( GL_COLOR_BUFFER_BIT );

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    for(const auto& window_component : window_components_) {
        std::string window_name = window_component.first;

        //ImGui::Begin( window_component.first.c_str(), &running_ );

        for(auto& priority_component : window_component.second){
            for(auto& component : priority_component.second){
                component->RenderInit();
            }
        }

        //ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
    glfwSwapBuffers( window );


    while( running_ ){

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        TimestampType ts;
        bool ready = true;

//        for(const auto& window_component : window_components_)
//        {
//            std::string window_name = window_component.first;
//            std::scoped_lock lock(data_lock_);
//            for(const auto& ts_ready : render_ready_[window_name]){
//                if(ts_ready.second.size() == all_render_component_names_.size()) {
//                    ts = ts_ready.first;
//                    ready = true;
//                    break;
//                }
//            }
//            if(ready)
//                break;
//        }

        if(ready)
        {
            glfwPollEvents();
            glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
            glClear( GL_COLOR_BUFFER_BIT );

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();


            std::scoped_lock lock(data_lock_);

            for(const auto& window_component : window_components_) {
                std::string window_name = window_component.first;


//                const auto window_ready_data = render_ready_[window_name].find(ts);
//                if(window_ready_data == render_ready_[window_name].end()){
//                    spdlog::error("could nto find ready flags for renderer");
//                    continue;
//                }
//                bool window_ready = window_ready_data->second.size() == all_render_component_names_.size();
//
//                if(!window_ready)
//                    continue;

                ImGui::Begin( window_name.c_str(), &running_ );

                for(auto& priority_component : window_component.second){
                    for(auto& component : priority_component.second){
                        component->Draw(ts);
                    }
                }

                ImGui::End();

                //render_ready_[window_name].erase(window_ready_data);
            }


            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
            glfwSwapBuffers( window );
        }



        running_ = !glfwWindowShouldClose(window);
    }

    ImGui_ImplGlfw_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

traact::component::render::RenderModule::RenderModule() {

}

void traact::component::render::RenderModule::setComponentReady(const std::string &window_name,
                                                                const std::string &component_name,
                                                                traact::TimestampType ts, bool valid) {
    std::scoped_lock lock(data_lock_);
    render_ready_[window_name][ts][component_name] = valid;

}

traact::component::render::RenderComponent::RenderComponent(const std::string &name) : ModuleComponent(name,ComponentType::SyncSink, ModuleType::Global) {

}

std::string traact::component::render::RenderComponent::GetModuleKey() {
    return "GlobalRenderModule";
}

traact::component::Module::Ptr traact::component::render::RenderComponent::InstantiateModule() {
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

void traact::component::render::RenderComponent::invalidTimePoint(traact::TimestampType ts, std::size_t mea_idx) {
    render_module_->setComponentReady(window_name_, getName(), ts, false);
    releaseAsyncCall(ts);
}
