/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
                                                                         traact::component::ComponentType::AsyncSource) {
            running_ = false;
        }

        traact::pattern::Pattern::Ptr GetPattern()  const {


            traact::pattern::Pattern::Ptr
                    pattern = getUncalibratedCameraPattern();
            pattern->name = "OpenCVVideoCapture";

            return pattern;
        }

        bool start() override {
            running_ = true;
            spdlog::info("starting OpenCV_VideoCapture");
            thread_.reset(new std::thread(std::bind(&OpenCVVideoCapture::threadLoop, this)));
            return true;
        }
        bool stop() override {
            spdlog::info("stopping OpenCV_VideoCapture");
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
                spdlog::error("Cannot open camera");
                return;
            }

            while (running_) {

                cv::Mat image;
                cap.read(image);
                TimestampType ts = now();

                SPDLOG_INFO("new image width: {0}", image.cols);
                SPDLOG_INFO("new image height: {0}", image.rows);


                auto buffer_future = request_callback_(ts);
                buffer_future.wait();
                auto buffer = buffer_future.get();
                if(!buffer){
                    SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
                    continue;
                }
                auto &newData = buffer->getOutput<ImageHeader::NativeType, ImageHeader>(0);

                if(!newData.IsCpu() && !newData.IsGpu()){
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

                spdlog::trace("commit data");
                buffer->Commit(true);
                spdlog::trace("data commited");
            }
            spdlog::trace("source quit loop");
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