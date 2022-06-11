/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <sstream>
#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>
namespace traact::component {

class Pose6DPrint : public traact::component::Component {
 public:
    explicit Pose6DPrint(const std::string &name) : traact::component::Component(name) {
        lastTimestamp = traact::Timestamp::min();
    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("Pose6DPrint", Concurrency::SERIAL,
                                                       traact::component::ComponentType::SYNC_SINK);

        pattern->addConsumerPort("input", Pose6DHeader::MetaType);

        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("A", "B", "input");

        return pattern;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::spatial;
        const auto &input = data.getInput<Pose6DHeader>(0);

        traact::Timestamp ts = data.getTimestamp();
        if (ts < lastTimestamp) {
            SPDLOG_WARN("current ts: {0} < lastTs: {1}",
                         ts.time_since_epoch().count(),
                         lastTimestamp.time_since_epoch().count());
        }

        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        std::stringstream ss;
        ss << input.matrix().format(CleanFmt);

        SPDLOG_INFO("{0} ts: {1}, value: \n{2}", "Pose6DPrint", ts.time_since_epoch().count(), ss.str());

        lastTimestamp = ts;

        return true;

    }
 protected:
    traact::Timestamp lastTimestamp;


};

CREATE_TRAACT_COMPONENT_FACTORY(Pose6DPrint)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Pose6DPrint)
END_TRAACT_PLUGIN_REGISTRATION
