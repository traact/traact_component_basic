/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
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
                                                                               traact::component::ComponentType::SYNC_SINK) {

    }

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::spatial;
        traact::pattern::spatial::SpatialPattern::Ptr
            pattern =
            std::make_shared<traact::pattern::spatial::SpatialPattern>("PrintBodyList", UNLIMITED);

        pattern->addConsumerPort("input", BodyListHeader::MetaType);

        return pattern;
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::spatial;
        const auto &input = data.getInput<BodyListHeader::NativeType, BodyListHeader>(0);

        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        std::stringstream ss;
        for (const Body &body : input) {
            ss << "Body: " << body.id;
            for (const auto &joint : body.bodyJoints) {
                ss << joint.second.pose.matrix().format(CleanFmt);
            }

        }

        spdlog::info("got result for ts: {0}, value: {1}", data.getTimestamp().time_since_epoch().count(), ss.str());

        return true;

    }

};

}

#endif //TRAACTMULTI_TRAACT_SPATIAL_SRC_COMPONENT_PRINTBODYLIST_H_
