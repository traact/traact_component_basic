/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/




#include <rttr/registration>
#include <traact/traact.h>
#include <fmt/format.h>
#include <traact/spatial.h>
namespace traact::component {

class StaticPose : public Component {
 public:
    using OutPort = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit StaticPose(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        std::string pattern_name = fmt::format("StaticPose");

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>(pattern_name, Concurrency::SERIAL, ComponentType::SYNC_SOURCE);

        pattern->addProducerPort<OutPort>("output");

        pattern->addParameter("tx", 0);
        pattern->addParameter("ty", 0);
        pattern->addParameter("tz", 0);

        pattern->addParameter("rx", 0);
        pattern->addParameter("ry", 0);
        pattern->addParameter("rz", 0);
        pattern->addParameter("rw", 1);

        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("A", "B", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        double tx, ty, tz, rx, ry, rz, rw;
        pattern::setValueFromParameter(pattern_instance, "tx", tx, 0);
        pattern::setValueFromParameter(pattern_instance, "ty", ty, 0);
        pattern::setValueFromParameter(pattern_instance, "tz", tz, 0);

        pattern::setValueFromParameter(pattern_instance, "rx", rx, 0);
        pattern::setValueFromParameter(pattern_instance, "ry", ry, 0);
        pattern::setValueFromParameter(pattern_instance, "rz", rz, 0);
        pattern::setValueFromParameter(pattern_instance, "rw", rw, 1);

        pose_.setIdentity();
        pose_.translate(Eigen::Vector3d(tx, ty, tz));
        pose_.rotate(Eigen::Quaterniond(rw, rx, ry, rz));

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        auto &output = data.getOutput<OutPort>();
        output = pose_;
        return true;
    }

 protected:

    Eigen::Affine3d pose_;



};

CREATE_TRAACT_COMPONENT_FACTORY(StaticPose)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::StaticPose)
END_TRAACT_PLUGIN_REGISTRATION
