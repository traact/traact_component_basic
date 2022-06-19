/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>
#include <traact/vision/outside_in/PointEstimation.h>

namespace traact::component::tracking {
class OutsideInPointEstimation : public Component {
 public:
    using InPortGroupInputPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupInputCalibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;
    using InPortGroupInputPointList = buffer::PortConfig<traact::vision::KeyPointListHeader, 2>;
    using InPortGroupInputPointListFeature = buffer::PortConfig<traact::vision::FeatureListHeader, 3>;

    using OutPortPosition3DList = buffer::PortConfig<traact::vision::Position3DListHeader, 0>;
    using OutPortPosition3DListFeature = buffer::PortConfig<traact::vision::FeatureListHeader, 1>;

    explicit OutsideInPointEstimation(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OutsideInPointEstimation",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addProducerPort<OutPortPosition3DList>("output")
            .addProducerPort<OutPortPosition3DListFeature>("output_feature")
            .beginPortGroup("camera", 2)
            .addConsumerPort<InPortGroupInputPose>("pose")
            .addConsumerPort<InPortGroupInputCalibration>("calibration")
            .addConsumerPort<InPortGroupInputPointList>("points")
            .addConsumerPort<InPortGroupInputPointListFeature>("points_feature")
            .endPortGroup();

        return pattern;
    }

    virtual void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        group_info_ = pattern_instance.getPortGroupInfo("camera");
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        return processTimePointWithInvalid(data);
    }
    bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override {

        std::vector<const spatial::Pose6D *> cameras2world;
        std::vector<const vision::CameraCalibration *> calibrations;
        std::vector<const vision::KeyPointList *> points;
        cameras2world.reserve(group_info_.size);
        calibrations.reserve(group_info_.size);
        points.reserve(group_info_.size);
        for (int group_instance = 0; group_instance < group_info_.size; ++group_instance) {
            if (data.isInputGroupValid(group_info_.port_group_index, group_instance)) {
                cameras2world.emplace_back(&data.getInput<InPortGroupInputPose>(group_info_.port_group_index,
                                                                                group_instance));
                calibrations.emplace_back(&data.getInput<InPortGroupInputCalibration>(group_info_.port_group_index,
                                                                                      group_instance));
                points.emplace_back(&data.getInput<InPortGroupInputPointList>(group_info_.port_group_index,
                                                                              group_instance));
            }
        }

        auto &output = data.getOutput<OutPortPosition3DList>();
        output.clear();
        std::vector<std::map<size_t, size_t>> output_to_group_point_index;
        vision::outside_in::estimatePoints(cameras2world, calibrations, points, output, &output_to_group_point_index);
        SPDLOG_DEBUG("found points {0}", output.size());

        for (const auto &p : output) {
            SPDLOG_DEBUG("point: {0} {1} {2}", p.x, p.y, p.z);
        }

        if (output.empty()) {
            data.setOutputInvalid<OutPortPosition3DList>();
        } else {
            auto &output_feature = data.getOutput<OutPortPosition3DListFeature>();
            output_feature.createIds(output.size());

            for (size_t model_index = 0; model_index < output.size(); ++model_index) {
                for (auto [group_instance, point_index] : output_to_group_point_index[model_index]) {
                    if (data.isInputGroupValid(group_info_.port_group_index, group_instance)) {
                        auto &input_feature =
                            data.getInput<InPortGroupInputPointListFeature>(group_info_.port_group_index, group_instance);
                        output_feature.constructed_from[output_feature.feature_id.at(model_index)].emplace_back(input_feature.feature_id[point_index]);
                    }

                }
            }
        }

        return true;
    }

 private:

    PortGroupInfo group_info_;

};

CREATE_TRAACT_COMPONENT_FACTORY(OutsideInPointEstimation)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::tracking::OutsideInPointEstimation)
END_TRAACT_PLUGIN_REGISTRATION
