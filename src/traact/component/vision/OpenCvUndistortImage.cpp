/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/UndistortionHelper.h>

namespace traact::component::opencv {

class OpenCvUndistortImage : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    using OutPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;

    explicit OpenCvUndistortImage(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvUndistortImage",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortCalibration>("input_calibration")
            .addProducerPort<OutPortImage>("output")
            .addProducerPort<OutPortCalibration>("output_calibration")
            .addParameter("optimizeIntrinsics", false)
            .addParameter("centerPrinciplePoint", false)
            .addParameter("alpha", 1.0, 0.0, 1.0)
            .addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "input_calibration")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("optimizeIntrinsics", optimize_intrinsics_);
        pattern_instance.setValueFromParameter("centerPrinciplePoint", center_principle_point_);
        pattern_instance.setValueFromParameter("alpha", alpha_);
        undistortion_.reset();
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<InPortImage>().value();
        const auto &calibration = data.getInput<InPortCalibration>();
        const auto &input_header = data.getInputHeader<InPortImage>();

        auto &output = data.getOutput<OutPortImage>().value();
        auto &output_header = data.getOutputHeader<OutPortImage>();
        output_header.copyFrom(input_header);
        auto &output_calibration = data.getOutput<OutPortCalibration>();

        undistortion_.init(calibration, optimize_intrinsics_, center_principle_point_, alpha_);
        output_calibration = undistortion_.getUndistortedCalibration();

        return undistortion_.undistortImage(image, output);
    }

 private:
    ::traact::vision::UndistortionHelper undistortion_{};
    bool optimize_intrinsics_{false};
    bool center_principle_point_{false};
    double alpha_{1.0};

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvUndistortImage)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvUndistortImage)
END_TRAACT_PLUGIN_REGISTRATION

