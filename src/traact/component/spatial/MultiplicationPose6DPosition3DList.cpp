/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include "traact/spatial.h"
#include <rttr/registration>
#include <traact/vision.h>
namespace traact::component {


class MultiplicationPose6DPosition3DList : public Component {
 public:
    using InPortA = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortB = buffer::PortConfig<traact::vision::Position3DListHeader, 1>;
    using OutPort = buffer::PortConfig<traact::vision::Position3DListHeader, 0>;

    explicit MultiplicationPose6DPosition3DList(const std::string &name) : Component(name) {}

    static pattern::Pattern::Ptr GetPattern(){
        using namespace traact::spatial;
        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("MultiplicationPose6DPosition3DList", traact::Concurrency::UNLIMITED, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortA>("input_a")
            .addConsumerPort<InPortB>("input_b")
            .addProducerPort<OutPort>("output");

        pattern->addCoordinateSystem("A", false)
            .addCoordinateSystem("B", false)
            .addCoordinateSystem("C", true)
            .addEdge("A", "B", "input0")
            .addEdge("B", "C", "input1")
            .addEdge("A", "C", "output");

        return
            pattern;
    };

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &input0 = data.getInput<InPortA>();
        const auto &input1 = data.getInput<InPortB>();
        auto &output = data.getOutput<OutPort>();

        output.resize(input1.size());
        for (size_t i = 0; i < input1.size(); ++i) {
            output[i] = input0 * input1[i];
        }

        return true;
    }


};
CREATE_TRAACT_COMPONENT_FACTORY(MultiplicationPose6DPosition3DList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::MultiplicationPose6DPosition3DList)
END_TRAACT_PLUGIN_REGISTRATION
