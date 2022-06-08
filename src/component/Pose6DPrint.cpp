/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <sstream>
#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>
namespace traact::component::spatial::util {

class Pose6DPrint : public traact::DefaultComponent {
 public:
    explicit Pose6DPrint(const std::string &name) : traact::DefaultComponent(name,
                                                                             traact::component::ComponentType::SYNC_SINK) {
        lastTimestamp = traact::Timestamp::min();
    }

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::spatial;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("Pose6DPrint", Concurrency::SERIAL);

        pattern->addConsumerPort("input", Pose6DHeader::MetaType);

        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("A", "B", "input");

        return pattern;
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
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