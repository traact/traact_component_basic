/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/opencv/OpenCVUtils.h>
#include <traact/vision.h>

#include <traact/math/perspective.h>

namespace traact::component {

class OutsideInPoseEstimation : public Component {
 public:
    using InPortModel = buffer::PortConfig<vision::Position3DListHeader, 0>;
    using InPortModelFeature = buffer::PortConfig<vision::FeatureListHeader, 1>;
    using InPortPosition3DListFeature = buffer::PortConfig<vision::FeatureListHeader, 2>;

    using InPortGroupInputPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupInputCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 1>;
    using InPortGroupInputPointList = buffer::PortConfig<vision::KeyPointListHeader, 2>;
    using InPortGroupInputPointListFeature = buffer::PortConfig<vision::FeatureListHeader, 3>;

    using OutPort_Pose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit OutsideInPoseEstimation(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OutsideInPoseEstimation",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortModel>("input")
            .addConsumerPort<InPortModelFeature>("input_feature")
            .addConsumerPort<InPortPosition3DListFeature>("input_points_feature")
            .beginPortGroup("camera", 2)
            .addConsumerPort<InPortGroupInputPose>("pose")
            .addConsumerPort<InPortGroupInputCalibration>("calibration")
            .addConsumerPort<InPortGroupInputPointList>("points")
            .addConsumerPort<InPortGroupInputPointListFeature>("points_feature")
            .endPortGroup()
            .addProducerPort<OutPort_Pose>("output")
            .addParameter("max_error", 5.0)
            .addParameter("min_points", 6);
        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        cameras_info_ = pattern_instance.getPortGroupInfo("camera");
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("max_error", max_error_);
        pattern_instance.setValueFromParameter("min_points", min_points_);

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        return processTimePointWithInvalid(data);
    }
    bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override {

        if(!data.isInputValid<InPortModel>() || !data.isInputValid<InPortModelFeature>() || !data.isInputValid<InPortPosition3DListFeature>()){
            return true;
        }
        const auto &model_points = data.getInput<InPortModel>();
        const auto &model_features = data.getInput<InPortModelFeature>();
        const auto &point_features = data.getInput<InPortPosition3DListFeature>();

        if(point_features.feature_id.size() < min_points_){
            SPDLOG_WARN("need at least {0} model points for pose estimation, currently found {1}", min_points_, point_features.feature_id.size());
            return true;
        }

        std::unordered_map<vision::FeatureID, size_t> point_2D_feature_to_model_index;
        for (int model_index = 0; model_index < model_features.constructed_from.size(); ++model_index) {

            auto model_feature = model_features.feature_id[model_index];
            auto find_point_3D_feature = model_features.constructed_from.find(model_feature);
            if(find_point_3D_feature == model_features.constructed_from.end()){
                SPDLOG_DEBUG("model feature {0} currently not found", model_index);
                continue;
            }
            if(find_point_3D_feature->second.size() != 1) {
                SPDLOG_ERROR("model feature should be reconstructed from exactly one 3d point feature");
                return true;
            }
            auto& point_3D_feature = find_point_3D_feature->second.front();
            auto find_point_2D_feature = point_features.constructed_from.find(point_3D_feature);
            if(find_point_2D_feature == point_features.constructed_from.end()){
                SPDLOG_ERROR("could not find point 2D features a 3D point was reconstructed from");
                return true;
            }
            for(auto point_2D_feature : find_point_2D_feature->second){
                point_2D_feature_to_model_index.emplace(point_2D_feature, model_index);
            }
        }

        std::vector<vision::Position2DList> local_image_point;
        local_image_point.reserve(cameras_info_.size);
        std::vector<traact::spatial::Pose6D> local_camera_to_world;
        local_camera_to_world.reserve(cameras_info_.size);
        std::vector<vision::CameraCalibration> local_camera_calibration;
        local_camera_calibration.reserve(cameras_info_.size);
        std::vector<vision::Position3DList> local_model_points;
        local_model_points.reserve(cameras_info_.size);


        for (size_t instance_index = 0; instance_index < cameras_info_.size; ++instance_index) {
            if (!data.isInputGroupValid(cameras_info_.port_group_index, instance_index)) {
                continue;
            }
            const auto &pose = data.getInput<InPortGroupInputPose>(cameras_info_.port_group_index, instance_index);
            const auto &calibration =
                data.getInput<InPortGroupInputCalibration>(cameras_info_.port_group_index, instance_index);
            const auto
                &point_list = data.getInput<InPortGroupInputPointList>(cameras_info_.port_group_index, instance_index);
            const auto &point_list_feature =
                data.getInput<InPortGroupInputPointListFeature>(cameras_info_.port_group_index, instance_index);

            vision::Position2DList point_data;
            point_data.reserve(model_points.size());
            vision::Position3DList model_data;
            model_data.reserve(model_points.size());
            for (size_t point_index = 0; point_index < point_list.size(); ++point_index) {
                auto was_used = point_2D_feature_to_model_index.find(point_list_feature.feature_id[point_index]);
                if(was_used != point_2D_feature_to_model_index.end()){
                    point_data.emplace_back(point_list[point_index].pt);
                    model_data.emplace_back(model_points[was_used->second]);
                }

            }

            if(!point_data.empty()){
                local_image_point.emplace_back(point_data);
                local_model_points.emplace_back(model_data);
                local_camera_to_world.emplace_back(pose);
                local_camera_calibration.emplace_back(calibration);
            }
        }

        if(!local_camera_to_world.empty()){
            auto &output = data.getOutput<OutPort_Pose>();
            auto valid_result = math::estimatePose6D(output, local_camera_to_world, local_camera_calibration, local_image_point, local_model_points, nullptr);

            if(!valid_result){
                data.setOutputInvalid<OutPort_Pose>();
                return true;
            }

            std::vector<vision::Position3DList> world_points(model_points.size());
            for (int camera_index = 0; camera_index < local_model_points.size(); ++camera_index) {
                world_points[camera_index].resize(local_model_points[camera_index].size());
                for (int point_index = 0; point_index < local_model_points[camera_index].size(); ++point_index) {
                    world_points[camera_index][point_index] = output * local_model_points[camera_index][point_index];
                }
            }


            auto error = math::reprojectionError(local_camera_to_world, local_image_point, local_camera_calibration, world_points);
            if(error > max_error_){
                data.setOutputInvalid<OutPort_Pose>();
            }

        }

        return true;
    }

 private:
    PortGroupInfo cameras_info_;
    Scalar max_error_;
    int min_points_;

};

CREATE_TRAACT_COMPONENT_FACTORY(OutsideInPoseEstimation)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::OutsideInPoseEstimation)
END_TRAACT_PLUGIN_REGISTRATION
