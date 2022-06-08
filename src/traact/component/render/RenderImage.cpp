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
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderImage", Concurrency::SERIAL);

        pattern->addConsumerPort("input", ImageHeader::MetaType);
        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("Camera", "ImagePlane", "input");

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

    bool start() override {
        last_ts_ = nowSteady();
        return ModuleComponent::start();
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::vision;
        const auto& input = data.getInput<ImageHeader>(0);

        auto now_steady = nowSteady();
        std::chrono::duration<double> diff = now_steady - last_ts_;
        last_ts_ = now_steady;
        fps_ = fps_ * 0.6 + 0.4 * 1.0 / diff.count();

        auto input_mat = input.GetCpuMat();
        cv::Mat image;
        if (input_mat.channels() == 4)
            image = input_mat.clone();
        else if (input_mat.channels() == 3)
            cv::cvtColor(input.GetCpuMat(), image, cv::COLOR_RGB2RGBA);
        else
            cv::cvtColor(input.GetCpuMat(), image, cv::COLOR_GRAY2RGBA);
        auto command = std::make_shared<RenderCommand>(window_name_, getName(),
                                                       data.getTimestamp().time_since_epoch().count(), priority_,
                                                       [this, image] { Draw(image); });

        render_module_->setComponentReady(command);

        return true;

    }

    void RenderInit() override {

    }

    void Draw(cv::Mat image) {
        //std::scoped_lock lock(data_lock_);


        if (!init_texture_) {
            glGenTextures(1, &texture_);
            init_texture_ = true;
        }

        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
        ImVec2 avail_size = ImGui::GetContentRegionAvail();
        if(avail_size.x < 10){
          avail_size.x = 320;
          avail_size.y = 180;
        }

        render_module_->setImageRenderSize(avail_size);



        ImGui::Image(reinterpret_cast<void *>( static_cast<intptr_t>( texture_ )), avail_size);

        ImGui::Begin("Stats");
        ImGui::Text("%2f",  fps_);
        ImGui::End();
    }

 private:
    GLuint texture_;
    bool init_texture_{false};
    double fps_{0};
    TimestampSteady last_ts_{TimestampSteady::min()};


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