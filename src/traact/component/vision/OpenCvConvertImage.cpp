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

namespace traact::component::vision {

class OpenCvConvertImage : public Component {
 public:
    explicit OpenCvConvertImage(const std::string &name) : Component(name,
                                                                     traact::component::ComponentType::SYNC_FUNCTIONAL) {
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvConvertImage", Concurrency::SERIAL);

        pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
        pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);

        pattern->addParameter("alpha", 255.0 / 2000.0);
        pattern->addParameter("beta", 0);

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(parameter, "alpha", alpha_, 255.0 / 2000.0);
        pattern::setValueFromParameter(parameter, "beta", beta_, 0);
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<ImageHeader>(0);
        auto &output = data.getOutput<ImageHeader>(0);

        image.GetCpuMat().convertTo(output.GetCpuMat(), CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), alpha_, beta_);

        return true;
    }

 private:
    double alpha_;
    double beta_;

 RTTR_ENABLE(Component)

};

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCvConvertImage>("OpenCvConvertImage").constructor<std::string>()();
}