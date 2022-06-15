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
#include <traact/vision/FLOutsideInTracking.h>

namespace traact::component {

class FeaturelessOutsideInTracking : public Component {
 public:
    explicit FeaturelessOutsideInTracking(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("FeaturelessOutsideInTracking", Concurrency::SERIAL,ComponentType::SYNC_SINK);

        pattern->addConsumerPort("input_model", traact::spatial::Position3DListHeader::NativeTypeName)
            .addProducerPort("output", traact::spatial::Pose6DHeader::NativeTypeName)
            .addProducerPort("output_points3d", traact::spatial::Position3DListHeader::NativeTypeName)

            .addCoordinateSystem("Target")
            .addCoordinateSystem("TargetPoints")
            .addCoordinateSystem("Origin")
            .addEdge("Target", "TargetPoints", "input_model")
            .addEdge("Origin", "Target", "output")
            .addEdge("Origin", "TargetPoints", "output_points3d")

            .beginPortGroup("camera_inputs", 0, 0)
            .addConsumerPort("input_{0}", traact::spatial::Position2DListHeader::NativeTypeName)
            .addConsumerPort("input_camera2world_{0}", traact::spatial::Pose6DHeader::NativeTypeName)
            .addConsumerPort("input_calibration_{0}", traact::vision::CameraCalibrationHeader::NativeTypeName)
            .addProducerPort("output_points_{0}", traact::spatial::Position2DListHeader::NativeTypeName)
            .addCoordinateSystem("Camera_{0}")
            .addCoordinateSystem("ImagePlane_{0}")
            .addEdge("ImagePlane_{0}", "TargetPoints", "input_{0}")
            .addEdge("Camera_{0}", "Origin", "input_camera2world_{0}")
            .addEdge("ImagePlane_{0}", "Camera_{0}", "input_calibration_{0}")
            .addEdge("ImagePlane_{0}", "TargetPoints", "output_points_{0}")
            .endPortGroup();

        pattern->addParameter("count_cameras", 2);

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(pattern_instance, "count_cameras", count_cameras_, 2);

        cameras_.resize(count_cameras_);
        tracking_.SetCountCameras(count_cameras_);
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;

        const auto &model_points = data.getInput<spatial::Position3DListHeader>(0);
        auto &output = data.getOutput<traact::spatial::Pose6DHeader>(0);
        auto &output_points3d = data.getOutput<traact::spatial::Position3DListHeader>(1);

        std::vector<spatial::Position2DList *> output_points(count_cameras_);

        for (int i = 0; i < count_cameras_; ++i) {
            const auto &points2d = data.getInput<spatial::Position2DListHeader>(i * 3 + 1);
            const auto &camera2world = data.getInput<spatial::Pose6DHeader>(i * 3 + 2);
            const auto &calibration = data.getInput<CameraCalibrationHeader>(i * 3 + 3);

            auto &output_tmp = data.getOutput<traact::spatial::Position2DListHeader>(i + 2);

            output_points[i] = &output_tmp;
            tracking_.SetData(i, &camera2world, &calibration, &points2d);

        }
        tracking_.Compute();
        bool valid_result = tracking_.FindTarget(model_points, output, &output_points, &output_points3d);
        if (!valid_result)
            return false;

        //output_points3d = tracking_.Get3DPoints();

        return true;

    }

 private:
    int count_cameras_;

    traact::vision::FLOutsideInTracking tracking_;
    std::vector<traact::vision::TrackingCamera> cameras_;

    Eigen::Affine3d prev_pose;
    bool pose_found_{false};



};

}


