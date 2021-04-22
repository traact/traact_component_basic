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


#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/highgui.hpp>
#include <traact/component/vision/OpenCVModule.h>
#include <rttr/registration>

namespace traact::component::vision {



    class SyncOpenCvMultiWindow : public OpenCVComponent {
    public:
        SyncOpenCvMultiWindow(const std::string &name)
                : OpenCVComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::vision;
            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("SyncOpenCvMultiWindow", serial);

            pattern->addConsumerPort("input", ImageHeader::MetaType);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            OpenCVComponent::configure(parameter, data);

            input_count = 4;
            for(int i = 0; i < input_count; ++i){
                opencv_module_->addComponent(getName() + "_" + std::to_string(i + 1), nullptr, 0);
            }
            return true;
        }


        bool start() override {
            start_ts_ = now();
            return ModuleComponent::start();
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::vision;
            SPDLOG_DEBUG("multi window {0} ", getName());
            auto inputCount = data.GetInputCount();
            for(int i = 0; i < inputCount; ++i){
                //const auto input = data.borrowInput<ImageHeader::NativeType, ImageHeader>(i);
                //render_module_->updateWindow(getName() + "_" + std::to_string(i + 1), data.getTimestamp().time_since_epoch().count(), input);
            }


            counter++;
            using nanoMilliseconds = std::chrono::duration<float, std::milli>;
            TimeDurationType time_diff = now() - start_ts_;
            float seconds = nanoMilliseconds(time_diff).count() / 1000.0;
            SPDLOG_INFO("{0} avg fps: {1}", getName(), counter/seconds);


            return true;

        }

        void invalidTimePoint(TimestampType ts, std::size_t mea_idx) override {
            counter++;
            for(int i = 0; i < input_count; ++i){
                //render_module_->updateWindow(getName() + "_" + std::to_string(i+1), nullptr);
            }

        }

        size_t counter{0};
        TimestampType start_ts_;
        int input_count;

        void Draw(cv::Mat &image) override {

        }


    RTTR_ENABLE(Component, ModuleComponent, OpenCVComponent)

    };

}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::SyncOpenCvMultiWindow>("SyncOpenCvMultiWindow").constructor<std::string>()();
}