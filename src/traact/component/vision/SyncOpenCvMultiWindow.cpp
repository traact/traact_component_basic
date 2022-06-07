/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/


#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/highgui.hpp>
#include "OpenCVModule.h"
#include <rttr/registration>

namespace traact::component::vision {

class SyncOpenCvMultiWindow : public OpenCVComponent {
 public:
    SyncOpenCvMultiWindow(const std::string &name)
        : OpenCVComponent(name) {}

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("SyncOpenCvMultiWindow", Concurrency::SERIAL);

        pattern->addConsumerPort("input", ImageHeader::MetaType);

        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("Camera", "ImagePlane", "input");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        OpenCVComponent::configure(parameter, data);

        input_count = 4;
        for (int i = 0; i < input_count; ++i) {
            opencv_module_->addComponent(getName() + "_" + std::to_string(i + 1), nullptr, 0);
        }
        return true;
    }

    bool start() override {
        start_ts_ = now();
        return ModuleComponent::start();
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::vision;
        SPDLOG_DEBUG("multi window {0} ", getName());
        auto inputCount = data.getInputCount();
        for (int i = 0; i < inputCount; ++i) {
            //const auto input = data.borrowInput<ImageHeader::NativeType, ImageHeader>(i);
            //render_module_->updateWindow(getName() + "_" + std::to_string(i + 1), data.getTimestamp().time_since_epoch().count(), input);
        }

        counter++;
        using nanoMilliseconds = std::chrono::duration<float, std::milli>;
        TimeDuration time_diff = now() - start_ts_;
        float seconds = nanoMilliseconds(time_diff).count() / 1000.0;
        SPDLOG_INFO("{0} avg fps: {1}", getName(), counter / seconds);

        return true;

    }

    void invalidTimePoint(Timestamp ts, size_t mea_idx) override {
        counter++;
        for (int i = 0; i < input_count; ++i) {
            //render_module_->updateWindow(getName() + "_" + std::to_string(i+1), nullptr);
        }

    }

    size_t counter{0};
    Timestamp start_ts_;
    int input_count;

    void Draw(cv::Mat &image) override {

    }

 RTTR_ENABLE(Component, ModuleComponent, OpenCVComponent)

};

}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::SyncOpenCvMultiWindow>("SyncOpenCvMultiWindow").constructor<std::string>()();
}