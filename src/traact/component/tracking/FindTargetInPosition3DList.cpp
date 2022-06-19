/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>
#include <traact/vision/outside_in/FindTargetPoints.h>

namespace traact::component::tracking {

class FindTargetInPosition3DList : public Component {
 public:
    using InPortPoints3D = buffer::PortConfig<vision::Position3DListHeader, 0>;
    using InPortPoints3DFeature = buffer::PortConfig<vision::FeatureListHeader, 1>;
    using InPortModelPoints3D = buffer::PortConfig<vision::Position3DListHeader, 2>;
    using OutPortModelPoints3DFeature = buffer::PortConfig<vision::FeatureListHeader, 0>;
    explicit FindTargetInPosition3DList(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("FindTargetInPosition3DList",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPoints3D>("input")
            .addConsumerPort<InPortPoints3DFeature>("input_feature")
            .addConsumerPort<InPortModelPoints3D>("input_model")
            .addProducerPort<OutPortModelPoints3DFeature>("output");


        return pattern;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;

        const auto &points_3D = data.getInput<InPortPoints3D>();
        const auto &points_3D_feature = data.getInput<InPortPoints3DFeature>();
        const auto &points_3D_model = data.getInput<InPortModelPoints3D>();


        if (points_3D.size() < points_3D_model.size()) {
            return true;
        }

        find_target_points_.initTarget(points_3D_model);
        auto find_result = find_target_points_.findTarget(points_3D);
        if(find_result.has_value()){
            auto &output = data.getOutput<OutPortModelPoints3DFeature>();
            output.createIds(points_3D_model.size());

            for (size_t model_index = 0; model_index < points_3D_model.size(); ++model_index) {
                auto point_index = find_result.value()[model_index];
                output.constructed_from.emplace(output.feature_id.at(model_index), std::vector<FeatureID>{points_3D_feature.feature_id[point_index]});
            }
        }



        return true;

    }

 private:
    vision::outside_in::FindTargetPoints find_target_points_;

};

CREATE_TRAACT_COMPONENT_FACTORY(FindTargetInPosition3DList)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::tracking::FindTargetInPosition3DList)
END_TRAACT_PLUGIN_REGISTRATION

