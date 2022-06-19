/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "RenderModuleComponent.h"
#include <traact/math/perspective.h>

namespace traact::component::render {

class RenderPose6D : public RenderComponent {
 public:
    using InPortPose = buffer::PortConfig<spatial::Pose6DHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    RenderPose6D(const std::string &name)
        : RenderComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderPose6D", Concurrency::SERIAL,ComponentType::SYNC_SINK);

        pattern->addConsumerPort<InPortPose>("input");
        pattern->addConsumerPort<InPortCalibration>("input_calibration");
        pattern->addStringParameter("Window", "RenderWindow")
            .addParameter("Priority", 3000);
        pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Target")
            .addEdge("ImagePlane", "Camera", "input_calibration")
            .addEdge("Camera", "Target", "input");
        return pattern;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::spatial;

        auto& pose = data.getInput<InPortPose>();
        auto& calibration = data.getInput<InPortCalibration>();

        latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                       data.getTimestamp().time_since_epoch().count(), priority_,
                                                       [this, calibration, pose] { draw(calibration, pose); });
        render_module_->setComponentReady(latest_command_);

        return true;

    }

    void draw(vision::CameraCalibration calibration, spatial::Pose6D pose) {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImVec2 v_min = ImGui::GetWindowContentRegionMin();
        auto win_pos = ImGui::GetWindowPos();
        win_pos += v_min;

        auto content_max = ImGui::GetContentRegionAvail();
        auto image_size = render_module_->getImageRenderSize(window_name_);
        if(image_size.has_value()){
          content_max = image_size.value();
        }
        double scale_x = content_max.x / calibration.width;
        double scale_y = content_max.y / calibration.height;


        auto p0 = traact::math::reproject_point(pose, calibration,
                                                vision::Position3D (0, 0, 0));
        auto px = traact::math::reproject_point(pose, calibration,
                                                vision::Position3D (1, 0, 0));
        auto py = traact::math::reproject_point(pose, calibration,
                                                vision::Position3D (0, 1, 0));
        auto pz = traact::math::reproject_point(pose, calibration,
                                                vision::Position3D (0, 0, 1));

        ImVec2 p_0(win_pos.x + p0.x * scale_x, win_pos.y + p0.y * scale_y);

        draw_list->AddLine(p_0,
                           ImVec2(win_pos.x + px.x * scale_x, win_pos.y + px.y * scale_y),
                           ImColor(255, 0, 0),
                           2);
        draw_list->AddLine(p_0,
                           ImVec2(win_pos.x + py.x * scale_x, win_pos.y + py.y * scale_y),
                           ImColor(0, 255, 0),
                           2);
        draw_list->AddLine(p_0,
                           ImVec2(win_pos.x + pz.x * scale_x, win_pos.y + pz.y * scale_y),
                           ImColor(0, 0, 255),
                           2);
    }
};

CREATE_TRAACT_COMPONENT_FACTORY(RenderPose6D)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderPose6D)
END_TRAACT_PLUGIN_REGISTRATION