/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>

namespace traact::component::spatial::core {

class MultiplicationComponent : public Component {
 public:
    explicit MultiplicationComponent(const std::string &name) : Component(name, ComponentType::SYNC_FUNCTIONAL) {}

    pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::spatial;
        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("MultiplicationComponent", traact::Concurrency::UNLIMITED);

        pattern->addConsumerPort("input0", Pose6DHeader::MetaType)
            .addConsumerPort("input1", Pose6DHeader::MetaType)
            .addProducerPort("output", Pose6DHeader::MetaType);

        pattern->addCoordinateSystem("A", false)
            .addCoordinateSystem("B", false)
            .addCoordinateSystem("C", false)
            .addEdge("A", "B", "input0")
            .addEdge("B", "C", "input1")
            .addEdge("A", "C", "output");

        return
            pattern;
    };

    bool processTimePoint(DefaultComponentBuffer &data) override {
        const auto &input0 = data.getInput<traact::spatial::Pose6DHeader>(0);
        const auto &input1 = data.getInput<traact::spatial::Pose6DHeader>(1);
        auto &output = data.getOutput<traact::spatial::Pose6DHeader>(0);

        output = input0 * input1;

        return true;
    }

 RTTR_ENABLE(Component)
};
}


// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::spatial::core::MultiplicationComponent>("MultiplicationComponent").constructor<
        std::string>()();
}