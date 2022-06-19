/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#include "RenderModuleComponent.h"
#include <traact/math/perspective.h>

namespace traact::component::render {

class RenderPosition3DList : public RenderComponent {
 public:
    using InPortPoints = buffer::PortConfig<vision::Position3DListHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    explicit RenderPosition3DList(const std::string &name)
        : RenderComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderPosition3DList", Concurrency::SERIAL,ComponentType::SYNC_SINK);

        pattern->addConsumerPort<InPortPoints>("input");
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

        auto& pose = data.getInput<InPortPoints>();
        auto& calibration = data.getInput<InPortCalibration>();

        latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                          data.getTimestamp().time_since_epoch().count(), priority_,
                                                          [this, calibration, pose] { draw(calibration, pose); });
        render_module_->setComponentReady(latest_command_);

        return true;

    }

    void draw(vision::CameraCalibration calibration, vision::Position3DList points) {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        auto win_pos = ImGui::GetWindowPos()+ImGui::GetWindowContentRegionMin();

        auto render_size = render_module_->getImageRenderSize(window_name_);
        auto image_size = render_module_->getImageSize(window_name_);
        ImVec2 scale(1.0,1.0);
        if(image_size.has_value() && render_size.has_value()){
            scale = render_size.value() / image_size.value();
        }

        for (const auto &point : points) {
            auto p_image = traact::math::reproject_point(calibration, point);
            ImVec2 point_pos(p_image.x,p_image.y);
            point_pos = win_pos + point_pos * scale;
            draw_list->AddCircle(point_pos, 7, ImColor(0, 84, 240));
        }
//        ImGui::Begin("Renderer Stats");
//        ImGui::Text("%s points 3D: %ld",
//                    window_name_.c_str(),
//                    points.size());
//        ImGui::End();
    }
};

CREATE_TRAACT_COMPONENT_FACTORY(RenderPosition3DList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderPosition3DList)
END_TRAACT_PLUGIN_REGISTRATION