/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/CudaComponent.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace traact::component::opencv {

class OpenCvGpuConvertImage : public CudaComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    explicit OpenCvGpuConvertImage(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            CudaComponent::GetPattern("OpenCvGpuConvertImage", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addProducerPort<OutPortImage>("output")
            .addParameter("alpha", 255.0 / 2000.0, 0.0, 1.0)
            .addParameter("beta", 0.0, -255.0, 255.0);

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(pattern_instance, "alpha", alpha_, 255.0 / 2000.0);
        pattern::setValueFromParameter(pattern_instance, "beta", beta_, 0);
        SPDLOG_INFO("set convert parameter to alpha {0} beta {1}", alpha_, beta_);
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &image = data.getInput<InPortImage>().value();
        const auto &input_header = data.getInputHeader<InPortImage>();


        auto& output = data.getOutput<OutPortImage>().value();
        auto& output_header = data.getOutputHeader<OutPortImage>();
        output_header.copyFrom(input_header);
        output_header.pixel_format = vision::PixelFormat::LUMINANCE;
        output_header.base_type = BaseType::UINT_8;
        output.create(image.rows, image.cols, CV_8UC1);



        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data, this](cudaStream_t stream) {
            if(data->isInputValid<InPortImage>()){
                const auto &input = data->getInput<InPortImage>().value();
                auto &output = data->getOutput<OutPortImage>().value();

                auto cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);

                input.convertTo(output, CV_8U, alpha_, beta_, cv_stream);
            }

        };
    }

 private:
    traact::Scalar alpha_;
    traact::Scalar beta_;



};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvGpuConvertImage)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvGpuConvertImage)
END_TRAACT_PLUGIN_REGISTRATION
