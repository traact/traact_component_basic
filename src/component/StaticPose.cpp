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
#include <fmt/format.h>
#include <traact/spatial.h>
namespace traact::component {

    class StaticPose : public Component {
    public:
        explicit StaticPose(const std::string &name) : Component(name,traact::component::ComponentType::SyncSource){
        }



        traact::pattern::Pattern::Ptr GetPattern()  const {


            std::string pattern_name = fmt::format("StaticPose");

            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>(pattern_name, serial);

            pattern->addProducerPort("output", spatial::Pose6DHeader::MetaType);

            pattern->addParameter("tx",0);
            pattern->addParameter("ty",0);
            pattern->addParameter("tz",0);

            pattern->addParameter("rx",0);
            pattern->addParameter("ry",0);
            pattern->addParameter("rz",0);
            pattern->addParameter("rw",1);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            double tx,ty,tz,rx,ry,rz,rw;
            pattern::setValueFromParameter(parameter, "tx", tx, 0);
            pattern::setValueFromParameter(parameter, "ty", ty, 0);
            pattern::setValueFromParameter(parameter, "tz", tz, 0);

            pattern::setValueFromParameter(parameter, "rx", rx, 0);
            pattern::setValueFromParameter(parameter, "ry", ry, 0);
            pattern::setValueFromParameter(parameter, "rz", rz, 0);
            pattern::setValueFromParameter(parameter, "rw", rw, 1);

            pose_.setIdentity();
            pose_.translate(Eigen::Vector3d(tx,ty,tz));
            pose_.rotate(Eigen::Quaterniond(rw,rx,ry,rz));



            return true;
        }

        bool processTimePoint(buffer::ComponentBuffer &data) override {
            auto& output = data.getOutput<Eigen::Affine3d, spatial::Pose6DHeader>(0);
            output = pose_;
            return true;
        }


    protected:

        Eigen::Affine3d pose_;


    RTTR_ENABLE(Component)

    };

}
// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::StaticPose>("StaticPose").constructor<std::string>()();
}