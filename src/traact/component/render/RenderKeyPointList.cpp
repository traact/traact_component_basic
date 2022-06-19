/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "RenderModuleComponent.h"

namespace traact::component::render {

class RenderKeyPointList : public RenderComponent {
 public:
    using InPortPoints = buffer::PortConfig<vision::KeyPointListHeader, 0>;
    RenderKeyPointList(const std::string &name)
        : RenderComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderKeyPointList", Concurrency::SERIAL, ComponentType::SYNC_SINK);

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

    void Draw(const vision::KeyPointList & data) {
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        auto win_pos = ImGui::GetWindowPos()+ImGui::GetWindowContentRegionMin();

        auto render_size = render_module_->getImageRenderSize(window_name_);
        auto image_size = render_module_->getImageSize(window_name_);
        ImVec2 scale(1.0,1.0);
        if(image_size.has_value() && render_size.has_value()){
            scale = render_size.value() / image_size.value();
        }

        for (const auto &point : data) {
            ImVec2 point_pos(point.pt.x,point.pt.y);
            point_pos = win_pos + point_pos * scale;
            ImVec2 size = ImVec2(point.size/2,point.size/2)  * scale;
            draw_list->AddQuad(point_pos+ImVec2(1,1)*size, point_pos+ImVec2(1,-1)*size, point_pos+ImVec2(-1,-1)*size, point_pos+ImVec2(-1,1)*size,ImColor(255, 0, 0), 2 );
            //draw_list->AddCircle(point_pos, point.size*scale.x, ImColor(255, 0, 0));


        }



    }


};
CREATE_TRAACT_COMPONENT_FACTORY(RenderKeyPointList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderKeyPointList)
END_TRAACT_PLUGIN_REGISTRATION
