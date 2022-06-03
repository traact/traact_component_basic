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
#include <traact/math/perspective.h>

namespace traact::component::render {



    class RenderPose6D : public RenderComponent {
    public:
        RenderPose6D(const std::string &name)
                : RenderComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::spatial;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("RenderPose6D", serial);

            pattern->addConsumerPort("input", Pose6DHeader::MetaType);
            pattern->addConsumerPort("input_calibration", vision::CameraCalibrationHeader::MetaType);
            pattern->addStringParameter("window", "invalid");
            pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Target")
            .addEdge("ImagePlane","Camera","input_calibration")
            .addEdge("Camera","Target", "input");
            return pattern;
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::spatial;
            //std::scoped_lock lock(data_lock_);
            auto pose = data.getInput<Pose6DHeader::NativeType, Pose6DHeader>(0);
            auto calibration = data.getInput<vision::CameraCalibrationHeader::NativeType, vision::CameraCalibrationHeader>(1);

            auto command = std::make_shared<RenderCommand>(window_name_, getName(),
                                                           data.GetTimestamp().time_since_epoch().count()
                                                           , priority_,
                                                           [this, calibration, pose] { Draw(calibration, pose); });
            render_module_->setComponentReady(command);


            return true;

        }

        void RenderInit() override {

        }

        void Draw(vision::CameraCalibration calibration, spatial::Pose6D pose)  {
            //std::scoped_lock lock(data_lock_);
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            ImVec2 vMin = ImGui::GetWindowContentRegionMin();
            auto win_pos = ImGui::GetWindowPos();
            win_pos.x += vMin.x;
            win_pos.y += vMin.y;
            auto content_max = ImGui::GetWindowContentRegionMax();

            double scalex = content_max.x / calibration.width;
            double scaley = content_max.y / calibration.height;

            Eigen::Vector2d win_offset(win_pos.x, win_pos.y);

            auto p0 = traact::math::reproject_point(pose, calibration,
                                                    Eigen::Vector3d(0, 0, 0)) ;
            auto px = traact::math::reproject_point(pose, calibration,
                                                    Eigen::Vector3d(1, 0, 0));
            auto py = traact::math::reproject_point(pose, calibration,
                                                    Eigen::Vector3d(0, 1, 0));
            auto pz = traact::math::reproject_point(pose, calibration,
                                                    Eigen::Vector3d(0, 0, 1));




            ImVec2 p_0(win_pos.x+p0.x()*scalex,win_pos.y+p0.y()*scaley);

            draw_list->AddLine(p_0, ImVec2(win_pos.x+px.x()*scalex,win_pos.y+px.y()*scaley), ImColor(255,0,0),2);
            //draw_list->AddDrawCmd();
            draw_list->AddLine(p_0, ImVec2(win_pos.x+py.x()*scalex,win_pos.y+py.y()*scaley), ImColor(0,255,0),2);
            //draw_list->AddDrawCmd();
            draw_list->AddLine(p_0, ImVec2(win_pos.x+pz.x()*scalex,win_pos.y+pz.y()*scaley), ImColor(0,0,255),2);
            //draw_list->AddDrawCmd();



        }

    private:
        //spatial::Pose6D data_;
        //vision::CameraCalibration intrinsics_;
        //std::mutex data_lock_;


    RTTR_ENABLE(Component, ModuleComponent, RenderComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::render::RenderPose6D>("RenderPose6D").constructor<std::string>()();
}