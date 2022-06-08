/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/spatial.h>
#include "OpenCVModule.h"
#include <rttr/registration>
#include <opencv2/opencv.hpp>

namespace traact::component::vision {

class OpenCvPaint2DPositionList : public OpenCVComponent {
 public:
    OpenCvPaint2DPositionList(const std::string &name)
        : OpenCVComponent(name) {}

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvPaint2DPositionList", Concurrency::SERIAL);

        pattern->addConsumerPort("input", spatial::Position2DListHeader::MetaType);
        pattern->addStringParameter("window", "sink");
        pattern->addCoordinateSystem("ImagePlane").addCoordinateSystem("Points")
            .addEdge("ImagePlane", "Points", "input");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        OpenCVComponent::configure(parameter, data);
        pattern::setValueFromParameter(parameter, "window", window_name_, "sink");
        opencv_module_->addComponent(window_name_, this, 10);
        return true;
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::spatial;
        data_ = data.getInput<Position2DListHeader>(0);

        return true;

    }

    void Draw(cv::Mat &image) override {
        for (const auto &point : data_) {
            cv::circle(image, cv::Point2d(point.x(), point.y()), 4, cv::Scalar_(255, 0, 0), 1);
        }
    }

 private:
    traact::spatial::Position2DListHeader::NativeType data_;
    std::string window_name_;

 RTTR_ENABLE(Component, ModuleComponent, OpenCVComponent)

};

}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCvPaint2DPositionList>("OpenCvPaint2DPositionList").constructor<
        std::string>()();
}