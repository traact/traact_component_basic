/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "RenderModule.h"
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

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderPose6D", Concurrency::SERIAL,ComponentType::SYNC_SINK);

        pattern->addConsumerPort("input", Pose6DHeader::MetaType);
        pattern->addConsumerPort("input_calibration", vision::CameraCalibrationHeader::MetaType);
        pattern->addStringParameter("window", "invalid");
        pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Target")
            .addEdge("ImagePlane", "Camera", "input_calibration")
            .addEdge("Camera", "Target", "input");
        return pattern;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::spatial;
        //std::scoped_lock lock(data_lock_);
        auto pose = data.getInput<Pose6DHeader>(0);
        auto calibration = data.getInput<vision::CameraCalibrationHeader>(1);

        auto command = std::make_shared<RenderCommand>(window_name_, getName(),
                                                       data.getTimestamp().time_since_epoch().count(), priority_,
                                                       [this, calibration, pose] { Draw(calibration, pose); });
        render_module_->setComponentReady(command);

        return true;

    }

    void RenderInit() override {

    }

    void Draw(vision::CameraCalibration calibration, spatial::Pose6D pose) {
        //std::scoped_lock lock(data_lock_);
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImVec2 vMin = ImGui::GetWindowContentRegionMin();
        auto win_pos = ImGui::GetWindowPos();
        win_pos.x += vMin.x;
        win_pos.y += vMin.y;
        auto content_max = ImGui::GetContentRegionAvail();
        auto image_size = render_module_->getImageRenderSize();
        if(image_size.has_value()){
          content_max = image_size.value();
        }


        double scale_x = content_max.x / calibration.width;
        double scale_y = content_max.y / calibration.height;

        //win_pos.x -= vMin.x;
        //win_pos.y -= vMin.y;

        auto p0 = traact::math::reproject_point(pose, calibration,
                                                Eigen::Vector3d(0, 0, 0));
        auto px = traact::math::reproject_point(pose, calibration,
                                                Eigen::Vector3d(1, 0, 0));
        auto py = traact::math::reproject_point(pose, calibration,
                                                Eigen::Vector3d(0, 1, 0));
        auto pz = traact::math::reproject_point(pose, calibration,
                                                Eigen::Vector3d(0, 0, 1));

        ImVec2 p_0(win_pos.x + p0.x() * scale_x, win_pos.y + p0.y() * scale_y);

        draw_list->AddLine(p_0,
                           ImVec2(win_pos.x + px.x() * scale_x, win_pos.y + px.y() * scale_y),
                           ImColor(255, 0, 0),
                           2);
        //draw_list->AddDrawCmd();
        draw_list->AddLine(p_0,
                           ImVec2(win_pos.x + py.x() * scale_x, win_pos.y + py.y() * scale_y),
                           ImColor(0, 255, 0),
                           2);
        //draw_list->AddDrawCmd();
        draw_list->AddLine(p_0,
                           ImVec2(win_pos.x + pz.x() * scale_x, win_pos.y + pz.y() * scale_y),
                           ImColor(0, 0, 255),
                           2);
        //draw_list->AddDrawCmd();

        ImGui::Begin("Stats");
        ImGui::Text("content: %2f %2f",  content_max.x, content_max.y);
        ImGui::End();

    }

 private:
    //spatial::Pose6D data_;
    //vision::CameraCalibration intrinsics_;
    //std::mutex data_lock_;




};

CREATE_TRAACT_COMPONENT_FACTORY(RenderPose6D)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderPose6D)
END_TRAACT_PLUGIN_REGISTRATION