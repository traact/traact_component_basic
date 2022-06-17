/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/opencv/OpenCVUtils.h>

namespace traact::component::opencv {

class OpenCVUndistort2DPoints : public Component {
 public:
    using InPortPoints = buffer::PortConfig<vision::Position2DListHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    using InPortDesiredCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 2>;
    using OutPortPoints = buffer::PortConfig<vision::Position2DListHeader, 0>;
    explicit OpenCVUndistort2DPoints(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCVUndistort2DPoints", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPoints>("input")
        .addConsumerPort<InPortCalibration>("input_calibration")
        .addConsumerPort<InPortDesiredCalibration>("input_desired_calibration")
        .addProducerPort<OutPortPoints>("output");

        pattern->addCoordinateSystem("ImagePlane_Distorted")
            .addCoordinateSystem("ImagePlane_Undistorted")
            .addCoordinateSystem("Points")
            .addEdge("ImagePlane_Distorted", "ImagePlane_Undistorted", "input_calibration")
            .addEdge("ImagePlane_Distorted", "Points", "input")
            .addEdge("ImagePlane_Undistorted", "Points", "output");

        return pattern;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input = data.getInput<InPortPoints>();
        const auto &calibration = data.getInput<InPortCalibration>();
        const auto &desired_calibration = data.getInput<InPortDesiredCalibration>();
        auto &output = data.getOutput<OutPortPoints>();

        output.resize(input.size());
        if (input.empty())
            return true;

        cv::Mat cv_intrinsics;
        cv::Mat cv_distortion;
        traact2cv(calibration, cv_intrinsics, cv_distortion);

        cv::Mat cv_desired_intrinsics;
        cv::Mat cv_desired_distortion;
        traact2cv(desired_calibration, cv_desired_intrinsics, cv_desired_distortion);

        output.clear();
        output.reserve(input.size());
        cv::undistortPoints(input,
                            output,
                            cv_intrinsics,
                            cv_distortion,
                            cv::noArray(),
                            cv_desired_intrinsics);

        return true;
    }


};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCVUndistort2DPoints)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCVUndistort2DPoints)
END_TRAACT_PLUGIN_REGISTRATION

