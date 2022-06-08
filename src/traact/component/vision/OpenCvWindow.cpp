/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/highgui.hpp>
#include "OpenCVModule.h"
#include <rttr/registration>
#include <opencv2/opencv.hpp>

namespace traact::component::vision {

class OpenCvWindow : public OpenCVComponent {
 public:
    OpenCvWindow(const std::string &name)
        : OpenCVComponent(name) {}

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvWindow", Concurrency::SERIAL);

        pattern->addConsumerPort("input", ImageHeader::MetaType);
        pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane").addEdge("Camera",
                                                                                         "ImagePlane",
                                                                                         "input");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        OpenCVComponent::configure(parameter, data);
        opencv_module_->addComponent(getName(), this, 0);
        image_ = cv::Mat(64, 48, CV_8UC3);
        return true;
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::vision;
        const auto input = data.getInput<ImageHeader>(0);

        cv::cvtColor(input.GetCpuMat(), image_, cv::COLOR_GRAY2BGR);

        return true;

    }

    void Draw(cv::Mat &image) override {
        image = image_.clone();
    }

 private:
    cv::Mat image_;

 RTTR_ENABLE(Component, ModuleComponent, OpenCVComponent)

};

}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCvWindow>("OpenCvWindow").constructor<std::string>()();
}