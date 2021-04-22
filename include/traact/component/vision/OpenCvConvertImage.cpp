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

namespace traact::component::vision {

    class OpenCvConvertImage : public Component {
    public:
        explicit OpenCvConvertImage(const std::string &name) : Component(name,
                                                                           traact::component::ComponentType::Functional) {
        }

        traact::pattern::Pattern::Ptr GetPattern()  const {


            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("OpenCvConvertImage", serial);

            pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
            pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);

            pattern->addParameter("irToGray", 6000.0);


            pattern->addCoordianteSystem("ImagePlane")
                    .addCoordianteSystem("Image", true)
                    .addEdge("ImagePlane", "Image", "input")
                    .addEdge("ImagePlane", "Image", "output");

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            pattern::setValueFromParameter(parameter, "irToGray", ir_to_gray_, 6000.0);
            return true;
        }

        bool processTimePoint(buffer::ComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& image = data.getInput<ImageHeader::NativeType, ImageHeader>(0);
            auto& output = data.getOutput<ImageHeader::NativeType, ImageHeader>(0);

            image.GetCpuMat().convertTo(output.GetCpuMat(), CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), 255. / ir_to_gray_);

            return true;
        }


    private:
        double ir_to_gray_;

    RTTR_ENABLE(Component)

    };

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCvConvertImage>("OpenCvConvertImage").constructor<std::string>()();
}