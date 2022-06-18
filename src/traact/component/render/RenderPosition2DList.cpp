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
        const auto input = data.getInput<InPortPoints>();

        latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                       data.getTimestamp().time_since_epoch().count(), priority_,
                                                       [this, input] { Draw(input); });
        render_module_->setComponentReady(latest_command_);

        return true;

    }

    void Draw(vision::Position2DList data) {
        //std::scoped_lock lock(data_lock_);
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImVec2 vMin = ImGui::GetWindowContentRegionMin();
        auto win_pos = ImGui::GetWindowPos();
        win_pos.x += vMin.x + 0.5f;
        win_pos.y += vMin.y + 0.5f;

        for (const auto &point : data) {
            draw_list->AddCircle(ImVec2(win_pos.x + point.x, win_pos.y + point.y), 5, ImColor(255, 0, 0));
        }

    }

 private:
    //vision::Position2DList data_;
    //std::mutex data_lock_;




};
CREATE_TRAACT_COMPONENT_FACTORY(RenderPosition2DList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderPosition2DList)
END_TRAACT_PLUGIN_REGISTRATION
