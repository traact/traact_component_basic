/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>

namespace traact::component::opencv {

class OpenCvGpuDecode : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    explicit OpenCvGpuDecode(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvGpuDecode", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addProducerPort<OutPortImage>("output")
            ;

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {


        return true;
    }

 private:



};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvGpuDecode)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvGpuDecode)
END_TRAACT_PLUGIN_REGISTRATION
