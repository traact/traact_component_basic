/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>

namespace traact::component {

class Multiplication : public Component {
 public:
    explicit Multiplication(const std::string &name) : Component(name) {}

    static pattern::Pattern::Ptr GetPattern(){
        using namespace traact::spatial;
        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("MultiplicationComponent", traact::Concurrency::UNLIMITED, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort("input0", Pose6DHeader::NativeTypeName)
            .addConsumerPort("input1", Pose6DHeader::NativeTypeName)
            .addProducerPort("output", Pose6DHeader::NativeTypeName);

        pattern->addCoordinateSystem("A", false)
            .addCoordinateSystem("B", false)
            .addCoordinateSystem("C", false)
            .addEdge("A", "B", "input0")
            .addEdge("B", "C", "input1")
            .addEdge("A", "C", "output");

        return
            pattern;
    };

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &input0 = data.getInput<traact::spatial::Pose6DHeader>(0);
        const auto &input1 = data.getInput<traact::spatial::Pose6DHeader>(1);
        auto &output = data.getOutput<traact::spatial::Pose6DHeader>(0);

        output = input0 * input1;

        return true;
    }


};
CREATE_TRAACT_COMPONENT_FACTORY(Multiplication)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Multiplication)
END_TRAACT_PLUGIN_REGISTRATION
