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
#include <traact/vision/BruteForcePosePnP.h>

namespace traact::component::tracking {

class BruteForceEstimatePose : public Component {
 public:
    using InPortPoints2D = buffer::PortConfig<vision::KeyPointListHeader, 0>;
    using InPortModelPoints3D = buffer::PortConfig<vision::Position3DListHeader, 1>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 2>;
    using OutPortPose = buffer::PortConfig<spatial::Pose6DHeader, 0>;
    explicit BruteForceEstimatePose(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("BruteForceEstimatePose",
                                                       Concurrency::UNLIMITED,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPoints2D>("input")
            .addConsumerPort<InPortModelPoints3D>("input_model")
            .addConsumerPort<InPortCalibration>("input_calibration")
            .addProducerPort<OutPortPose>("output")
            .addParameter("maxPointDistance", 150.0)
            .addParameter("minError", 1.0)
            .addParameter("maxError", 1.0);

        pattern->addCoordinateSystem("Target")
            .addCoordinateSystem("TargetPoints")
            .addCoordinateSystem("Camera")
            .addCoordinateSystem("ImagePlane")
            .addEdge("ImagePlane", "TargetPoints", "input")
            .addEdge("Target", "TargetPoints", "input_model")
            .addEdge("ImagePlane", "Camera", "input_calibration")
            .addEdge("Camera", "Target", "output")
            .addEdge("Origin", "TargetPoints", "output_points3d")

            .addEdge("Camera_{0}", "Origin", "input_camera2world_{0}")
            .addEdge("ImagePlane_{0}", "TargetPoints", "output_points_{0}");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("maxPointDistance", max_point_distance_);
        pattern_instance.setValueFromParameter("minError", min_error_);
        pattern_instance.setValueFromParameter("maxError", max_error_);

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;

        const auto &points2d = data.getInput<InPortPoints2D>();
        const auto &points3d = data.getInput<InPortModelPoints3D>();
        const auto &calibration = data.getInput<InPortCalibration>();

        if (points2d.size() < points3d.size()) {
            return true;
        }

        auto &output = data.getOutput<OutPortPose>();
        //auto &output_points = data.getOutput<traact::vision::Position2DListHeader>(1);
        vision::Position2DList image_points;
        cv::KeyPoint::convert(points2d, image_points);

        Timestamp ts_start2 = now();

        bool result = false;

//        if(!last_successful_.empty()){
//            auto error = testCombination(image_points, points3d, calibration, output, last_successful_);
//            if(error < min_error_){
//                SPDLOG_INFO("found pose using last index list");
//                result = true;
//            }
//
//        }

        if(!result){
            result = tryClusterCombinations(
                image_points,
                points3d,
                calibration,
                output, min_error_, max_error_, max_point_distance_, &last_successful_);
        }
        TimeDuration dur2 = now() - ts_start2;
        SPDLOG_INFO("found pose {0} in {1}", result, std::chrono::duration_cast<std::chrono::milliseconds>(dur2).count());

        if (!result) {
            data.setOutputInvalid<OutPortPose>();
            last_successful_.clear();
        }

        return true;

    }

 private:
    Scalar max_point_distance_;
    Scalar min_error_;
    Scalar max_error_;
    std::vector<size_t> last_successful_;
};

CREATE_TRAACT_COMPONENT_FACTORY(BruteForceEstimatePose)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::tracking::BruteForceEstimatePose)
END_TRAACT_PLUGIN_REGISTRATION

