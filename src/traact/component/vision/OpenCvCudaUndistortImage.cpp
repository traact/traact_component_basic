/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/GpuUndistortionHelper.h>
#include <traact/component/CudaComponent.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace traact::component::opencv {

class OpenCvCudaUndistortImage : public CudaComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using InPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    using OutPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using OutPortCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;

    explicit OpenCvCudaUndistortImage(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("OpenCvCudaUndistortImage",
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
        const auto &input_header = data.getInputHeader<InPortImage>();
        const auto &calibration = data.getInput<InPortCalibration>();

        auto &output = data.getOutput<OutPortImage>().value();
        auto &output_header = data.getOutputHeader<OutPortImage>();
        auto &output_calibration = data.getOutput<OutPortCalibration>();

        output.create(image.size(), image.type());
        output_header.copyFrom(input_header);

        undistortion_.init(calibration,
                           optimize_intrinsics_,
                           center_principle_point_,
                           alpha_);

        output_calibration = undistortion_.getUndistortedCalibration();

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data, this](cudaStream_t stream) {
            if (data->isInputValid<InPortImage>()) {
                const auto &input = data->getInput<InPortImage>().value();
                const auto &calibration = data->getInput<InPortCalibration>();
                auto &output = data->getOutput<OutPortImage>().value();

                auto cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);

                undistortion_.init(cv_stream);


                if (!undistortion_.undistortImage(input, output, cv_stream)) {
                    data->setOutputInvalid<OutPortImage>();
                }

            }

        };
    }

 private:
    ::traact::vision::GpuUndistortionHelper undistortion_{};
    bool optimize_intrinsics_{false};
    bool center_principle_point_{false};
    double alpha_{1.0};

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvCudaUndistortImage)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvCudaUndistortImage)
END_TRAACT_PLUGIN_REGISTRATION

