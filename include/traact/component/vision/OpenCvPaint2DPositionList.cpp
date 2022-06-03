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
#include <traact/spatial.h>
#include <traact/component/vision/OpenCVModule.h>
#include <rttr/registration>
#include <opencv2/opencv.hpp>

namespace traact::component::vision {



    class OpenCvPaint2DPositionList : public OpenCVComponent {
    public:
        OpenCvPaint2DPositionList(const std::string &name)
                : OpenCVComponent(name) {}

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::vision;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("OpenCvPaint2DPositionList", serial);

            pattern->addConsumerPort("input", spatial::Position2DListHeader::MetaType);
            pattern->addStringParameter("window", "sink");
            pattern->addCoordinateSystem("ImagePlane").addCoordinateSystem("Points")
            .addEdge("ImagePlane","Points","input");

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            OpenCVComponent::configure(parameter, data);
            pattern::setValueFromParameter(parameter, "window", window_name_, "sink");
            opencv_module_->addComponent(window_name_, this, 10);
            return true;
        }


        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::spatial;
            data_ = data.getInput<Position2DListHeader::NativeType, Position2DListHeader>(0);

            return true;

        }

        void Draw(cv::Mat &image) override {
            for(const auto& point : data_){
                cv::circle(image, cv::Point2d(point.x(),point.y()), 4, cv::Scalar_(255,0,0), 1);
            }
        }


    private:
        traact::spatial::Position2DListHeader::NativeType data_;
        std::string window_name_;



    RTTR_ENABLE(Component, ModuleComponent, OpenCVComponent)

    };



}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCvPaint2DPositionList>("OpenCvPaint2DPositionList").constructor<std::string>()();
}