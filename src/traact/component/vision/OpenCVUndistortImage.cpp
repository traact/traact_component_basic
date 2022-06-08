/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/UndistortionHelper.h>

namespace traact::component::vision {

class OpenCVUndistortImage : public Component {
 public:
    explicit OpenCVUndistortImage(const std::string &name) : Component(name,
                                                                       traact::component::ComponentType::SYNC_FUNCTIONAL) {
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCVUndistortImage", Concurrency::SERIAL);

        pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
        pattern->addConsumerPort("input_calibration", traact::vision::CameraCalibrationHeader::MetaType);
        pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);
        pattern->addProducerPort("output_calibration", traact::vision::CameraCalibrationHeader::MetaType);

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "input_calibration")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<ImageHeader>(0);
        const auto &calibration = data.getInput<CameraCalibrationHeader>(1);
        auto &output = data.getOutput<ImageHeader>(0);
        auto &output_calibration = data.getOutput<CameraCalibrationHeader>(1);

        undistorter_.Init(calibration, true, false, 1);
        output_calibration = undistorter_.GetUndistortedCalibration();

        return undistorter_.UndistortImage(image, output);
    }

 private:
    ::traact::vision::UndistortionHelper undistorter_;

 RTTR_ENABLE(Component)

};

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCVUndistortImage>("OpenCVUndistortImage").constructor<std::string>()();
}