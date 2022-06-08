/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>

namespace traact::component::spatial::core {

class InversionComponent : public Component {
 public:
    explicit InversionComponent(const std::string &name) : Component(name, ComponentType::SYNC_FUNCTIONAL) {}

    pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::spatial;
        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("InversionComponent", Concurrency::UNLIMITED);

        pattern->addConsumerPort("input", Pose6DHeader::MetaType)
            .addProducerPort("output", Pose6DHeader::MetaType);

        pattern->addCoordinateSystem("A", false)
            .addCoordinateSystem("B", false)
            .addEdge("A", "B", "input")
            .addEdge("B", "A", "output");

        return
            pattern;
    };

    bool processTimePoint(DefaultComponentBuffer &data) override {
        const auto &input = data.getInput<traact::spatial::Pose6DHeader>(0);
        auto &output = data.getOutput<traact::spatial::Pose6DHeader>(0);

        output = input.inverse();

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
    registration::class_<traact::component::spatial::core::InversionComponent>("InversionComponent").constructor<std::string>()();
}