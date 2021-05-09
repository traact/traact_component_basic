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

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/render/RenderModule.h>
#include <rttr/registration>

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <opencv2/imgproc.hpp>
#include <mutex>

namespace traact::component::render {



//class RenderImage : public AsyncRenderComponent<traact::vision::ImageHeader::NativeType> {
class RenderImage : public RenderComponent {
    public:
        RenderImage(const std::string &name)
                : RenderComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::vision;
            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("RenderImage", serial);

            pattern->addConsumerPort("input", ImageHeader::MetaType);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            render_module_ = std::dynamic_pointer_cast<RenderModule>(module_);
            pattern::setValueFromParameter(parameter, "window", window_name_, getName());
            pattern::setValueFromParameter(parameter, "priority", priority_, 0);
            render_module_->addComponent(window_name_, getName(), this, priority_);

            //parameter["window"]["value"] = getName();
            //RenderComponent::configure(parameter, data);

            //render_module_->addComponent(getName(), getName(), this, 0);
            //image_ = cv::Mat(64,48,CV_8UC4);


            return true;
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::vision;
            const auto input = data.getInput<ImageHeader::NativeType, ImageHeader>(0);

//            {
//                std::scoped_lock lock(data_lock_);
//                data_.emplace(data.getTimestamp(), &input);
//            }
//            render_module_->setComponentReady(window_name_, getName(), data.getTimestamp(), true);
            //std::scoped_lock lock(data_lock_);
            //cv::cvtColor(input.GetCpuMat(), image_, cv::COLOR_GRAY2RGBA);
            cv::Mat image;
            cv::cvtColor(input.GetCpuMat(), image, cv::COLOR_GRAY2RGBA);
            auto command = std::make_shared<RenderCommand>(window_name_, getName(),
                                                           data.GetMeaIdx(), priority_,
                                                           [this, image] { Draw(image); });

            render_module_->setComponentReady(command);


            return true;

        }

    void RenderInit() override {

    }

    void Draw(cv::Mat image )  {
        //std::scoped_lock lock(data_lock_);


        if(!init_texture_){
            glGenTextures( 1, &texture_ );
            init_texture_ = true;
        }


            glBindTexture( GL_TEXTURE_2D, texture_ );
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
            glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
            glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data );
            ImVec2 avail_size = ImGui::GetContentRegionAvail();
            //ImGui::Image( reinterpret_cast<void*>( static_cast<intptr_t>( texture_ ) ), ImVec2( image_.cols, image_.rows ) );
            ImGui::Image( reinterpret_cast<void*>( static_cast<intptr_t>( texture_ ) ), avail_size );
        }

    private:
    GLuint texture_;
        bool init_texture_{false};
        //std::mutex data_lock_;
        //cv::Mat image_;


    RTTR_ENABLE(Component, ModuleComponent, RenderComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::render::RenderImage>("RenderImage").constructor<std::string>()();
}