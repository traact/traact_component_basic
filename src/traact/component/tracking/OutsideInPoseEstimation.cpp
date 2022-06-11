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

namespace traact::component {

class OutsideInPoseEstimation : public Component {
 public:
    using InPort_Model = buffer::PortConfig<traact::spatial::Position3DListHeader, 0>;

    using InPortGroupInputs_Pose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupInputs_Points = buffer::PortConfig<traact::spatial::Position2DListHeader, 1>;
    using InPortGroupInputs_Calibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 2>;

    using OutPort_Pose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit OutsideInPoseEstimation(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("EstimatePose", Concurrency::UNLIMITED, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPort_Model>("input_model");

        pattern->beginPortGroup("Inputs", 2)
            .addConsumerPort<InPortGroupInputs_Pose>("camera_pose")
            .addConsumerPort<InPortGroupInputs_Points>("camera_points")
            .addConsumerPort<InPortGroupInputs_Calibration>("camera_calibration")
            .endPortGroup();


        pattern->addProducerPort<OutPort_Pose> ("output");


        return pattern;
    }

};

CREATE_TRAACT_COMPONENT_FACTORY(OutsideInPoseEstimation)

}


BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::OutsideInPoseEstimation)
END_TRAACT_PLUGIN_REGISTRATION
