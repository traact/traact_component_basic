/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#include <rttr/registration>
#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <thread>
#include <traact/buffer/SourceComponentBuffer.h>
namespace traact::component::vision {

class OpenCVVideoCapture : public Component {
 public:
    explicit OpenCVVideoCapture(const std::string &name) : Component(name,
                                                                     traact::component::ComponentType::ASYNC_SOURCE) {
        running_ = false;
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        traact::pattern::Pattern::Ptr
            pattern = getUncalibratedCameraPattern();
        pattern->name = "OpenCVVideoCapture";

        return pattern;
    }

    bool start() override {
        running_ = true;
        SPDLOG_INFO("starting OpenCV_VideoCapture");
        thread_.reset(new std::thread(std::bind(&OpenCVVideoCapture::threadLoop, this)));
        return true;
    }
    bool stop() override {
        SPDLOG_INFO("stopping OpenCV_VideoCapture");
        if (running_) {
            running_ = false;
            thread_->join();
        }
        return true;
    }

 private:
    std::shared_ptr<std::thread> thread_;
    bool running_;

    void threadLoop() {
        using namespace traact::vision;
        using namespace traact;

        int output_count = 0;

        //auto cap = cv::VideoCapture(0, cv::CAP_V4L);
        auto cap = cv::VideoCapture("/dev/v4l/by-path/pci-0000:00:14.0-usb-0:11:1.0-video-index0", cv::CAP_V4L);

        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        if (!cap.isOpened()) {
            SPDLOG_ERROR("Cannot open camera");
            return;
        }

        while (running_) {

            cv::Mat image;
            cap.read(image);
            Timestamp ts = now();

            SPDLOG_INFO("new image width: {0}", image.cols);
            SPDLOG_INFO("new image height: {0}", image.rows);

            auto buffer_future = request_callback_(ts);
            buffer_future.wait();
            auto buffer = buffer_future.get();
            if (!buffer) {
                SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
                continue;
            }
            auto &newData = buffer->getOutput<ImageHeader::NativeType, ImageHeader>(0);

            if (!newData.IsCpu() && !newData.IsGpu()) {
                ImageHeader header;
                header.width = image.cols;
                header.height = image.rows;
                header.opencv_matrix_type = image.type();
                header.device_id = 0;
                newData.init(header);
                //image.copyTo(newData.GetCpuMat());
                newData.SetCpuMat(image);
            }

            /*if (newData.IsCpu())
              image.copyTo(newData.GetCpuMat());
            if (newData.IsGpu())
              newData.GetGpuMat().upload(image);*/

            SPDLOG_TRACE("commit data");
            buffer->commit(true);
            SPDLOG_TRACE("data commited");
        }
        SPDLOG_TRACE("source quit loop");
        running_ = false;
    }
 RTTR_ENABLE(Component)
};

}
// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCVVideoCapture>("OpenCVVideoCapture").constructor<std::string>()();
}