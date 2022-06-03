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

#include <sstream>
#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>
namespace traact::component::spatial::util {

    class Pose6DPrint : public traact::DefaultComponent {
    public:
        explicit Pose6DPrint(const std::string &name) : traact::DefaultComponent(name,
                                                                                 traact::component::ComponentType::SyncSink) {
            lastTimestamp = traact::TimestampType::min();
        }

        traact::pattern::Pattern::Ptr GetPattern() const {
            using namespace traact::spatial;
            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("Pose6DPrint", serial);

            pattern->addConsumerPort("input", Pose6DHeader::MetaType);

            pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("A","B","input");

            return pattern;
        }

        bool processTimePoint(traact::DefaultComponentBuffer &data) override {
            using namespace traact::spatial;
            const auto &input = data.getInput<Pose6DHeader::NativeType, Pose6DHeader>(0);

            traact::TimestampType ts = data.GetTimestamp();
            if (ts < lastTimestamp) {
                spdlog::warn("current ts: {0} < lastTs: {1}",
                             ts.time_since_epoch().count(),
                             lastTimestamp.time_since_epoch().count());
            }

            Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
            std::stringstream ss;
            ss << input.matrix().format(CleanFmt);


            spdlog::info("{0} ts: {1}, value: \n{2}", "Pose6DPrint", ts.time_since_epoch().count(), ss.str());

            lastTimestamp = ts;

            return true;

        }
    protected:
        traact::TimestampType lastTimestamp;

    RTTR_ENABLE(Component)
    };

}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::spatial::util::Pose6DPrint>("Pose6DPrint").constructor<std::string>()
            (
                    //policy::ctor::as_std_shared_ptr
            );
}