/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>

namespace traact::component {

class Inversion : public Component {
 public:
    explicit Inversion(const std::string &name) : Component(name) {}

    static pattern::Pattern::Ptr GetPattern(){
        using namespace traact::spatial;
        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("InversionPose6D", Concurrency::UNLIMITED, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort("input", Pose6DHeader::NativeTypeName)
            .addProducerPort("output", Pose6DHeader::NativeTypeName);

        pattern->addCoordinateSystem("A", false)
            .addCoordinateSystem("B", false)
            .addEdge("A", "B", "input")
            .addEdge("B", "A", "output");

        return
            pattern;
    };

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &input = data.getInput<traact::spatial::Pose6DHeader>(0);
        auto &output = data.getOutput<traact::spatial::Pose6DHeader>(0);

        output = input.inverse();

        return true;
    }


};

CREATE_TRAACT_COMPONENT_FACTORY(Inversion)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Inversion)
END_TRAACT_PLUGIN_REGISTRATION
