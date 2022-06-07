/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/


#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/opencv/OpenCVUtils.h>
namespace traact::component {

class OpenCVUndistort2DPoints : public Component {
 public:
    explicit OpenCVUndistort2DPoints(const std::string &name) : Component(name,
                                                                          traact::component::ComponentType::SYNC_FUNCTIONAL) {
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCVUndistort2DPoints", Concurrency::SERIAL);

        pattern->addConsumerPort("input", traact::spatial::Position2DListHeader::MetaType);
        pattern->addConsumerPort("input_calibration", traact::vision::CameraCalibrationHeader::MetaType);
        pattern->addProducerPort("output", traact::spatial::Position2DListHeader::MetaType);

        pattern->addCoordinateSystem("ImagePlane_Distorted")
            .addCoordinateSystem("ImagePlane_Undistorted")
            .addCoordinateSystem("Points")
            .addEdge("ImagePlane_Distorted", "ImagePlane_Undistorted", "input_calibration")
            .addEdge("ImagePlane_Distorted", "Points", "input")
            .addEdge("ImagePlane_Undistorted", "Points", "output");

        return pattern;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::spatial;
        using namespace traact::vision;
        const auto &input = data.getInput<Position2DListHeader>(0);
        const auto &calibration = data.getInput<CameraCalibrationHeader>(1);
        auto &output = data.getOutput<Position2DListHeader>(0);

        output.resize(input.size());
        if (input.empty())
            return true;

        cv::Mat cv_intrinsics;
        cv::Mat cv_distortion;
        traact2cv(calibration, cv_intrinsics, cv_distortion);
        std::vector<cv::Point2f> opencv_points;

        for (auto &point : input) {
            opencv_points.push_back(eigen2cv(point));
        }
        std::vector<cv::Point2f> opencv_points_output(opencv_points.size());
        cv::undistortPoints(opencv_points,
                            opencv_points_output,
                            cv_intrinsics,
                            cv_distortion,
                            cv::noArray(),
                            cv_intrinsics);

        for (int i = 0; i < opencv_points_output.size(); ++i) {
            output[i].x() = opencv_points_output[i].x;
            output[i].y() = opencv_points_output[i].y;
        }

        return true;
    }

 private:
    double alpha_;
    double beta_;

 RTTR_ENABLE(Component)

};

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::OpenCVUndistort2DPoints>("OpenCVUndistort2DPoints").constructor<std::string>()();
}