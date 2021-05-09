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
#include <mutex>

namespace traact::component::render {



    class RenderPosition2DList : public RenderComponent {
    public:
        RenderPosition2DList(const std::string &name)
                : RenderComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::spatial;
            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("RenderPosition2DList", serial);

            pattern->addConsumerPort("input", Position2DListHeader::MetaType);
            pattern->addStringParameter("window", "invalid");
            return pattern;
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::spatial;
            const auto input = data.getInput<Position2DListHeader::NativeType, Position2DListHeader>(0);

            //std::scoped_lock lock(data_lock_);
            //data_ = input;
            auto command = std::make_shared<RenderCommand>(window_name_, getName(),
                                                           data.GetMeaIdx(), priority_,
                                                           [this, input] { Draw(input); });
            render_module_->setComponentReady(command);

            return true;

        }

        void RenderInit() override {

        }


        void Draw(spatial::Position2DList data)  {
            //std::scoped_lock lock(data_lock_);
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 vMin = ImGui::GetWindowContentRegionMin();
            auto win_pos = ImGui::GetWindowPos();
            win_pos.x += vMin.x + 0.5f;
            win_pos.y += vMin.y+ 0.5f;


            for(const auto& point : data){
                draw_list->AddCircle(ImVec2(win_pos.x+point.x(), win_pos.y+point.y()), 5, ImColor(255,0,0));
            }


        }

    private:
        //spatial::Position2DList data_;
        //std::mutex data_lock_;


    RTTR_ENABLE(Component, ModuleComponent, RenderComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::render::RenderPosition2DList>("RenderPosition2DList").constructor<std::string>()();
}