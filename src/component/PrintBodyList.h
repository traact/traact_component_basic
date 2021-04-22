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

#ifndef TRAACTMULTI_TRAACT_SPATIAL_SRC_COMPONENT_PRINTBODYLIST_H_
#define TRAACTMULTI_TRAACT_SPATIAL_SRC_COMPONENT_PRINTBODYLIST_H_

#include <sstream>
#include <traact/traact.h>
#include "traact/spatialBody.h"

namespace traact::component::spatial::util {

class PrintBodyList : public traact::DefaultComponent {
 public:
  explicit PrintBodyList(const std::string &name) : traact::DefaultComponent(name,
                                                                           traact::component::ComponentType::SyncSink) {

  }

  traact::pattern::Pattern::Ptr GetPattern() const{
    using namespace traact::spatial;
    traact::pattern::spatial::SpatialPattern::Ptr
        pattern =
        std::make_shared<traact::pattern::spatial::SpatialPattern>("PrintBodyList", unlimited);

    pattern->addConsumerPort("input", BodyListHeader::MetaType);

    return pattern;
  }

  bool processTimePoint(traact::DefaultComponentBuffer &data) override {
    using namespace traact::spatial;
    const auto &input = data.getInput<BodyListHeader::NativeType, BodyListHeader>(0);



    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::stringstream ss;
    for(const Body& body : input) {
      ss << "Body: " << body.id;
      for(const auto& joint : body.bodyJoints) {
        ss << joint.second.pose.matrix().format(CleanFmt);
      }

    }


    spdlog::info("got result for ts: {0}, value: {1}", data.getTimestamp().time_since_epoch().count(), ss.str());


    return true;

  }

};

}

#endif //TRAACTMULTI_TRAACT_SPATIAL_SRC_COMPONENT_PRINTBODYLIST_H_
