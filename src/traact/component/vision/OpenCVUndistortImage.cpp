/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/UndistortionHelper.h>

namespace traact::component::opencv {

class OpenCVUndistortImage : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    using OutPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    explicit OpenCVUndistortImage(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCVUndistortImage",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortCalibration>("input_calibration")
            .addProducerPort<OutPortImage>("output")
            .addProducerPort<OutPortCalibration>("output_calibration")
            .addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "input_calibration")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<InPortImage>().getImage();
        const auto &calibration = data.getInput<InPortCalibration>();
        const auto &input_header = data.getInputHeader<InPortImage>();

        auto &output = data.getOutput<OutPortImage>().getImage();
        auto &output_header = data.getOutputHeader<OutPortImage>();
        output_header.copyFrom(input_header);
        auto& output_calibration = data.getOutput<OutPortCalibration>();

        undistorter_.Init(calibration, true, false, 1);
        output_calibration = undistorter_.GetUndistortedCalibration();

        return undistorter_.UndistortImage(image, output);
    }

 private:
    ::traact::vision::UndistortionHelper undistorter_;

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCVUndistortImage)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCVUndistortImage)
END_TRAACT_PLUGIN_REGISTRATION

