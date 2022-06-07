/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <opencv2/imgproc.hpp>

namespace traact::component::vision {

class OpenCvColorToGray : public Component {
 public:
    explicit OpenCvColorToGray(const std::string &name) : Component(name,
                                                                    traact::component::ComponentType::SYNC_FUNCTIONAL) {
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvColorToGray", Concurrency::SERIAL);

        pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
        pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input = data.getInput<ImageHeader>(0).GetCpuMat();
        auto &output = data.getOutput<ImageHeader>(0).GetCpuMat();

        auto channels = input.channels();
        auto depth = input.depth();
        cv::cvtColor(input, output, cv::COLOR_BGRA2GRAY);

        return true;
    }

 private:

 RTTR_ENABLE(Component)

};

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCvColorToGray>("OpenCvColorToGray").constructor<std::string>()();
}