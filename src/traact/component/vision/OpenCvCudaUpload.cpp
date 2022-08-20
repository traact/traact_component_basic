/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/CudaComponent.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace traact::component::opencv {

class OpenCvCudaUpload : public CudaComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    explicit OpenCvCudaUpload(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("OpenCvCudaUpload", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addProducerPort<OutPortImage>("output");

        pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "input")
            .addEdge("ImagePlane", "Image", "output");

        return pattern;
    }

    virtual bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &cpu_image = data.getInput<InPortImage>();
        auto &gpu_image = data.getOutput<OutPortImage>();
        data.getOutputHeader<OutPortImage>().copyFrom(data.getInputHeader<InPortImage>());

        const auto& cpu_mat = cpu_image.value();
        auto& gpu_mat = gpu_image.value();
        gpu_mat.create(cpu_mat.rows, cpu_mat.cols, cpu_mat.type());

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data](cudaStream_t stream) {

            if(data->isInputValid<InPortImage>()){
                const auto &cpu_image = data->getInput<InPortImage>().value();
                auto &gpu_image = data->getOutput<OutPortImage>().value();
                auto cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);
                gpu_image.upload(cpu_image, cv_stream);
            }

        };
    }

 private:

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvCudaUpload)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvCudaUpload)
END_TRAACT_PLUGIN_REGISTRATION
