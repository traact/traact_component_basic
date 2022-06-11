/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/opencv/OpenCVUtils.h>
#include <traact/vision/UndistortionHelper.h>
#include <traact/math/perspective.h>
#include <traact/spatial.h>
#include <iostream>
#include <fstream>

namespace traact::component::opencv {

class OpenCvArucoTracker : public Component {
 public:
    using InPort_Image = buffer::PortConfig<traact::vision::ImageHeader, 0>;
    using InPort_Calibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;

    using OutPortGroup_Pose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit OpenCvArucoTracker(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvArucoTracker", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPort_Image>("input")
            .addConsumerPort<InPort_Calibration>("input_calibration");

        pattern->beginPortGroup("outputs", 1, std::numeric_limits<int>::max())
            .addProducerPort<OutPortGroup_Pose>("output")
            .addParameter("MarkerId", 0, 0)
            .endPortGroup();

        pattern->addParameter("Dictionary", "DICT_4X4_50", {"DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50"})
            .addParameter("MarkerSize", 0.08);
        return pattern;
    }

 private:



};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvArucoTracker)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvArucoTracker)
END_TRAACT_PLUGIN_REGISTRATION
