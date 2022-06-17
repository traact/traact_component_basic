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

namespace traact::component::render {

class RenderPosition2DList : public RenderComponent {
 public:
    RenderPosition2DList(const std::string &name)
        : RenderComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderPosition2DList", Concurrency::SERIAL, ComponentType::SYNC_SINK);

        pattern->addConsumerPort("input", Position2DListHeader::NativeTypeName);
        pattern->addStringParameter("window", "invalid");
        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("ImagePlane", "Points", "input");
        return pattern;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::spatial;
        const auto input = data.getInput<Position2DListHeader>(0);

        //std::scoped_lock lock(data_lock_);
        //data_ = input;
        latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                       data.getTimestamp().time_since_epoch().count(), priority_,
                                                       [this, input] { Draw(input); });
        render_module_->setComponentReady(latest_command_);

        return true;

    }

    void renderInit() override {

    }

    void Draw(spatial::Position2DList data) {
        //std::scoped_lock lock(data_lock_);
        ImDrawList *draw_list = ImGui::GetWindowDrawList();
        ImVec2 vMin = ImGui::GetWindowContentRegionMin();
        auto win_pos = ImGui::GetWindowPos();
        win_pos.x += vMin.x + 0.5f;
        win_pos.y += vMin.y + 0.5f;

        for (const auto &point : data) {
            draw_list->AddCircle(ImVec2(win_pos.x + point.x(), win_pos.y + point.y()), 5, ImColor(255, 0, 0));
        }

    }

 private:
    //spatial::Position2DList data_;
    //std::mutex data_lock_;




};
CREATE_TRAACT_COMPONENT_FACTORY(RenderPosition2DList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderPosition2DList)
END_TRAACT_PLUGIN_REGISTRATION
