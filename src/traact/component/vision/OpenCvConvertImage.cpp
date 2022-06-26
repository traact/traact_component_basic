/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>

namespace traact::component::opencv {

class OpenCvConvertImage : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    explicit OpenCvConvertImage(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvConvertImage", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
        .addProducerPort<OutPortImage>("output")
        .addParameter("alpha", 255.0 / 2000.0)
        .addParameter("beta", 0.0);

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(pattern_instance, "alpha", alpha_, 255.0 / 2000.0);
        pattern::setValueFromParameter(pattern_instance, "beta", beta_, 0);
        SPDLOG_INFO("set convert parameter to alpha {0} beta {1}", alpha_, beta_);
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &image = data.getInput<InPortImage>().getImage();
        const auto &input_header = data.getInputHeader<InPortImage>();


        auto& output = data.getOutput<OutPortImage>().getImage();
        auto& output_header = data.getOutputHeader<OutPortImage>();
        output_header.copyFrom(input_header);
        output_header.pixel_format = vision::PixelFormat::LUMINANCE;
        output_header.base_type = BaseType::UINT_8;

        image.convertTo(output, CV_8U, alpha_, beta_);

        return true;
    }

 private:
    traact::Scalar alpha_;
    traact::Scalar beta_;



};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvConvertImage)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvConvertImage)
END_TRAACT_PLUGIN_REGISTRATION
