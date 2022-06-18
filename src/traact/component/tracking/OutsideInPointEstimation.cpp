/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>
#include <traact/vision/outside_in/PointEstimation.h>

namespace traact::component::tracking {
class OutsideInPointEstimation : public Component {
 public:
    using InPortGroupInputs_Pose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupInputs_Calibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;
    using InPortGroupInputs_Points = buffer::PortConfig<traact::vision::KeyPointListHeader, 2>;


    using OutPort_Positions = buffer::PortConfig<traact::vision::Position3DListHeader, 0>;

    explicit OutsideInPointEstimation(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("EstimatePose", Concurrency::UNLIMITED, ComponentType::SYNC_FUNCTIONAL);

        pattern->addProducerPort<OutPort_Positions> ("output");

        pattern->beginPortGroup("input", 2)
            .addConsumerPort<InPortGroupInputs_Pose>("camera_pose")
            .addConsumerPort<InPortGroupInputs_Calibration>("camera_calibration")
            .addConsumerPort<InPortGroupInputs_Points>("camera_points")
            .endPortGroup();

        return pattern;
    }

    virtual void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        group_info_ = pattern_instance.getPortGroupInfo("input");
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        return processTimePointWithInvalid(data);
    }
    bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override {

        std::vector<const spatial::Pose6D *> world_2_cameras;
        std::vector<const vision::CameraCalibration*> calibrations;
        std::vector<const vision::KeyPointList *> points;
        world_2_cameras.reserve(group_info_.size);
        calibrations.reserve(group_info_.size);
        points.reserve(group_info_.size);
        for (int group_instance = 0; group_instance < group_info_.size; ++group_instance) {
            if(data.isInputGroupValid(group_info_.port_group_index, group_instance)){
                world_2_cameras.emplace_back(&data.getInput<InPortGroupInputs_Pose>(group_info_.port_group_index, group_instance));
                calibrations.emplace_back(&data.getInput<InPortGroupInputs_Calibration>(group_info_.port_group_index, group_instance));
                points.emplace_back(&data.getInput<InPortGroupInputs_Points>(group_info_.port_group_index, group_instance));
            }
        }

        point_estimation_.

        point_estimation_.setCountCameras(0);

        return true;
    }

 private:
    vision::outside_in::PointEstimation point_estimation_;
    PortGroupInfo group_info_;

};

CREATE_TRAACT_COMPONENT_FACTORY(OutsideInPointEstimation)

}


BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::tracking::OutsideInPointEstimation)
END_TRAACT_PLUGIN_REGISTRATION
