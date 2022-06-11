/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <opencv2/imgproc.hpp>

namespace traact::component::opencv {

class OpenCvColorToGray : public Component {
 public:
    using InPort = buffer::PortConfig<traact::vision::ImageHeader, 0>;
    using OutPort = buffer::PortConfig<traact::vision::ImageHeader, 0>;

    explicit OpenCvColorToGray(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvColorToGray",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPort>("input")
            .addProducerPort<OutPort>("output")
            .addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto& input = data.getInput<InPort>().getImage();
        const auto& input_header = data.getInputHeader<InPort>();

        auto &output = data.getOutput<OutPort>().getImage();
        auto & output_header = data.getOutputHeader<InPort>();
        output_header.width = input_header.width;
        output_header.height = input_header.height;
        output_header.stride = input_header.stride;
        output_header.pixel_format = vision::PixelFormat::LUMINANCE;
        output_header.channels = 1;
        output_header.base_type = BaseType::UINT_8;

        cv::cvtColor(input, output, cv::COLOR_BGRA2GRAY);

        return true;
    }

 private:

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvColorToGray)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvColorToGray)
END_TRAACT_PLUGIN_REGISTRATION
