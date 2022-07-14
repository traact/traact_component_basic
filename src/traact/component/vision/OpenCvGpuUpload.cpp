/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/GpuComponent.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace traact::component::opencv {

class OpenCvGpuUpload : public GpuComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    explicit OpenCvGpuUpload(const std::string &name) : GpuComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = GpuComponent::GetPattern("OpenCvGpuUpload", Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

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

    GpuTask createGpuTask(buffer::ComponentBuffer *data) override {
        SPDLOG_INFO("create gpu task");
        return [data](cudaStream_t stream) {

            if(data->isInputValid<InPortImage>()){
                const auto &cpu_image = data->getInput<InPortImage>().value();
                auto &gpu_image = data->getOutput<OutPortImage>().value();

//                auto status = cudaMemcpy2DAsync(gpu_image.data, gpu_image.step, cpu_image.data, cpu_image.step, cpu_image.cols, cpu_image.rows, cudaMemcpyHostToDevice, stream);
//                if(status != cudaSuccess){
//                    SPDLOG_ERROR("unable to upload to gpu");
//                    data->setOutputInvalid<OutPortImage>();
//                }
                auto cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);
                gpu_image.upload(cpu_image, cv_stream);
            }

        };
    }

 private:

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvGpuUpload)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvGpuUpload)
END_TRAACT_PLUGIN_REGISTRATION
