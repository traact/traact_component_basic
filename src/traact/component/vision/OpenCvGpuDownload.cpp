/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/component/CudaComponent.h>
#include <opencv2/core/cuda_stream_accessor.hpp>
namespace traact::component::opencv {

class OpenCvGpuDownload : public CudaComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    explicit OpenCvGpuDownload(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            CudaComponent::GetPattern("OpenCvGpuDownload", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addProducerPort<OutPortImage>("output")
            ;

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto& gpu_image = data.getInput<InPortImage>().value();
        auto& cpu_image = data.getOutput<OutPortImage>().value();

        cpu_image.create(gpu_image.size(), gpu_image.type());

        data.getOutputHeader<OutPortImage>().copyFrom(data.getInputHeader<InPortImage>());

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data](cudaStream_t stream) {
            if(data->isInputValid<InPortImage>()){
                const auto &gpu_image = data->getInput<InPortImage>().value();
                auto &cpu_image = data->getOutput<OutPortImage>().value();

//                auto status = cudaMemcpy2DAsync(cpu_image.data, cpu_image.step, gpu_image.data, gpu_image.step, gpu_image.cols, gpu_image.rows, cudaMemcpyDeviceToHost, stream);
//                if(status != cudaSuccess){
//                    SPDLOG_ERROR("unable to download to cpu");
//                    data->setOutputInvalid<OutPortImage>();
//                }
                auto cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);
                gpu_image.download(cpu_image, cv_stream);
            }

        };
    }

 private:



};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvGpuDownload)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvGpuDownload)
END_TRAACT_PLUGIN_REGISTRATION
