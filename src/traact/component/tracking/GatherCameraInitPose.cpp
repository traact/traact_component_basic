/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>

namespace traact::component::tracking {
class GatherCameraInitPose : public Component {
 public:
    using InPortGroupInputPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using OutPortGroupInputPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit GatherCameraInitPose(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("GatherCameraInitPose", Concurrency::UNLIMITED, ComponentType::SYNC_FUNCTIONAL);

        pattern->beginPortGroup("Camera", 1)
            .addConsumerPort<InPortGroupInputPose>("input")
            .addProducerPort<OutPortGroupInputPose>("output")
            .endPortGroup();


        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        group_info_ = pattern_instance.getPortGroupInfo("Camera");
        world_to_camera_.resize(group_info_.size);
    }
    bool processTimePoint(buffer::ComponentBuffer &data) override {
        return processTimePointWithInvalid(data);
    }

    bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override {

        std::vector<std::optional<spatial::Pose6D> > current_camera_to_target;
        current_camera_to_target.resize(group_info_.size);


        for (int group_instance_index = 0; group_instance_index < group_info_.size; ++group_instance_index) {
            if(data.isInputValid<InPortGroupInputPose>(group_info_.port_group_index, group_instance_index)){
                current_camera_to_target[group_instance_index] = data.getInput<InPortGroupInputPose>(group_info_.port_group_index, group_instance_index);
            }
        }

        auto world_to_target = getCurrentWorldToTarget(current_camera_to_target);

        if(world_to_target.has_value()){
            for (int group_instance_index = 0; group_instance_index < group_info_.size; ++group_instance_index) {
                if(!world_to_camera_[group_instance_index].has_value() && current_camera_to_target[group_instance_index].has_value()){
                    world_to_camera_[group_instance_index] = world_to_target.value() * current_camera_to_target[group_instance_index]->inverse();
                }
            }
        }

        for (int group_instance_index = 0; group_instance_index < group_info_.size; ++group_instance_index) {
            if(world_to_camera_[group_instance_index].has_value()){
                auto& output = data.getOutput<OutPortGroupInputPose>(group_info_.port_group_index, group_instance_index);
                output = world_to_camera_[group_instance_index].value().inverse();
            }
        }

        return true;
    }

 private:
    PortGroupInfo group_info_;
    std::vector<std::optional<spatial::Pose6D> > world_to_camera_;
    bool has_init_pose_{false};

    std::optional<spatial::Pose6D> getCurrentWorldToTarget(const std::vector<std::optional<spatial::Pose6D>> &camera_to_target) {
        // compute with first already initialized pose

        for (int camera_index = 0; camera_index < group_info_.size; ++camera_index) {
            if(world_to_camera_[camera_index].has_value() && camera_to_target[camera_index].has_value()){
                return world_to_camera_[camera_index].value() * camera_to_target[camera_index].value();
            }
        }

        // some camera defined the origin, but is currently not tracking the target
        if(has_init_pose_){
            return {};
        } else {
            // define the first tracked target as the origin
            for (int camera_index = 0; camera_index < group_info_.size; ++camera_index) {
                if(camera_to_target[camera_index].has_value()){
                    world_to_camera_[camera_index] =  camera_to_target[camera_index]->inverse();
                    has_init_pose_ = true;
                    return spatial::Pose6D::Identity();
                }
            }
        }
        return {};
    }

};

CREATE_TRAACT_COMPONENT_FACTORY(GatherCameraInitPose)

}


BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::tracking::GatherCameraInitPose)
END_TRAACT_PLUGIN_REGISTRATION
