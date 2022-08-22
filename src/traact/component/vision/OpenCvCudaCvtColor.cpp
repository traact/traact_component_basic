/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/component/CudaComponent.h>
#include <opencv2/core/cuda_stream_accessor.hpp>
namespace traact::component::opencv {

class OpenCvCudaCvtColor : public CudaComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    explicit OpenCvCudaCvtColor(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            CudaComponent::GetPattern("OpenCvCudaCvtColor", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addProducerPort<OutPortImage>("output")
            .addParameter("output_type","CV_8UC1", {"CV_8UC1","CV_8UC3","CV_8UC4"});
            ;

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        std::string type_name;
        pattern_instance.setValueFromParameter("output_type", type_name);
        if(type_name == "CV_8UC1"){
            output_type_ = CV_8UC1;
        }
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto& input_header = data.getInputHeader<InPortImage>();

        auto& output_header =data.getOutputHeader<OutPortImage>();
        auto& output_image = data.getOutput<OutPortImage>().value();

        output_image.create(input_header.height, input_header.width, output_type_);

        output_header.setFrom(output_image);
        switch (output_header.channels) {
            case 1:{
                output_header.pixel_format = vision::PixelFormat::LUMINANCE;
            }
            case 3:{
                output_header.pixel_format = vision::PixelFormat::RGB;
            }
            case 4:{
                output_header.pixel_format = vision::PixelFormat::RGBA;
            }
            default:{
                output_header.pixel_format = vision::PixelFormat::UNKNOWN_PIXELFORMAT;
            }

        };

        //TODO construce ColorConversionCode from input and output headers
        cvt_code_ = cv::ColorConversionCodes::COLOR_BGRA2GRAY;

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data, cvt_code = cvt_code_](cudaStream_t stream) {
            if(data->isInputValid<InPortImage>()){
                const auto &input_image = data->getInput<InPortImage>().value();
                auto &output_image = data->getOutput<OutPortImage>().value();

                auto cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);
                cv::cuda::cvtColor(input_image, output_image, cvt_code, 0, cv_stream );
            }

        };
    }

 private:
    int output_type_;
    int cvt_code_;



};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvCudaCvtColor)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvCudaCvtColor)
END_TRAACT_PLUGIN_REGISTRATION
