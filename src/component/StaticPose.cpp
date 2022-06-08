/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/




#include <rttr/registration>
#include <traact/traact.h>
#include <fmt/format.h>
#include <traact/spatial.h>
namespace traact::component {

class StaticPose : public Component {
 public:
    explicit StaticPose(const std::string &name) : Component(name, traact::component::ComponentType::SYNC_SOURCE) {
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        std::string pattern_name = fmt::format("StaticPose");

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>(pattern_name, Concurrency::SERIAL);

        pattern->addProducerPort("output", spatial::Pose6DHeader::MetaType);

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

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        double tx, ty, tz, rx, ry, rz, rw;
        pattern::setValueFromParameter(parameter, "tx", tx, 0);
        pattern::setValueFromParameter(parameter, "ty", ty, 0);
        pattern::setValueFromParameter(parameter, "tz", tz, 0);

        pattern::setValueFromParameter(parameter, "rx", rx, 0);
        pattern::setValueFromParameter(parameter, "ry", ry, 0);
        pattern::setValueFromParameter(parameter, "rz", rz, 0);
        pattern::setValueFromParameter(parameter, "rw", rw, 1);

        pose_.setIdentity();
        pose_.translate(Eigen::Vector3d(tx, ty, tz));
        pose_.rotate(Eigen::Quaterniond(rw, rx, ry, rz));

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        auto &output = data.getOutput<spatial::Pose6DHeader>(0);
        output = pose_;
        return true;
    }

 protected:

    Eigen::Affine3d pose_;

 RTTR_ENABLE(Component)

};

}
// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::StaticPose>("StaticPose").constructor<std::string>()();
}