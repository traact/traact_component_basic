/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "RenderModuleComponent.h"

namespace traact::component::render {

class RenderPosition2DList : public RenderComponent {
 public:
    using InPortPoints = buffer::PortConfig<vision::Position2DListHeader, 0>;
    RenderPosition2DList(const std::string &name)
        : RenderComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderPosition2DList", Concurrency::SERIAL, ComponentType::SYNC_SINK);

        pattern->addConsumerPort<InPortPoints>("input")
            .addStringParameter("Window", "RenderWindow")
            .addParameter("Priority", 2000);

        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("ImagePlane", "Points", "input");
        return pattern;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::spatial;
        const auto& input = data.getInput<InPortPoints>();

        latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                       data.getTimestamp().time_since_epoch().count(), priority_,
                                                       [this, input] { Draw(input); });
        render_module_->setComponentReady(latest_command_);

        return true;

    }

    void Draw(vision::Position2DList data) {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        auto win_pos = ImGui::GetWindowPos()+ImGui::GetWindowContentRegionMin();

        auto render_size = render_module_->getImageRenderSize(window_name_);
        auto image_size = render_module_->getImageSize(window_name_);
        ImVec2 scale(1.0,1.0);
        if(image_size.has_value() && render_size.has_value()){
            scale = render_size.value() / image_size.value();
        }

        for (const auto &point : data) {
            ImVec2 point_pos(point.x,point.y);
            point_pos = win_pos + point_pos * scale;
            draw_list->AddCircle(point_pos, 5, ImColor(125, 54, 0));
        }
    }


};
CREATE_TRAACT_COMPONENT_FACTORY(RenderPosition2DList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderPosition2DList)
END_TRAACT_PLUGIN_REGISTRATION
