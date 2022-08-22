/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/aruco.hpp>
#include <traact/opencv/OpenCVUtils.h>

namespace traact::component::opencv {

class OpenCvArucoTracker : public Component {
 public:
    using InPortImage = buffer::PortConfig<traact::vision::ImageHeader, 0>;
    using InPortCalibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;

    using OutPortGroupPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using OutPortGroupDebug = buffer::PortConfig<traact::vision::ImageHeader, 0>;

    explicit OpenCvArucoTracker(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvArucoTracker",
                                                       Concurrency::UNLIMITED,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortCalibration>("input_calibration");

        pattern->beginPortGroup("output_pose", 1, std::numeric_limits<int>::max())
            .addProducerPort<OutPortGroupPose>("output")
            .addParameter("marker_id", 0, 0)
            .endPortGroup();
        pattern->beginPortGroup("output_debug", 0, 1)
            .addProducerPort<OutPortGroupDebug>("output")
            .endPortGroup();

        pattern->addParameter("Dictionary", "DICT_4X4_50", {"DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50"})
            .addParameter("marker_size", 0.08);
        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        int output_pose_count = pattern_instance.getPortGroupCount("output_pose");

        pose_port_group_ = pattern_instance.getPortGroupInfo("output_pose");
        debug_port_group_ = pattern_instance.getPortGroupInfo("output_debug");

    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override {
        cv::aruco::PREDEFINED_DICTIONARY_NAME dict;
        pattern::setValueFromParameter(pattern_instance,
                                       "Dictionary",
                                       dict,
                                       "DICT_4X4_50",
                                       {{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                                        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                                        {"DICT_6X6_50", cv::aruco::DICT_6X6_50}});
        pattern::setValueFromParameter(pattern_instance, "marker_size", marker_size_, 0.08);

        dictionary_ = cv::aruco::getPredefinedDictionary(dict);
        parameter_ = cv::aruco::DetectorParameters::create();

        const auto& pose_groups = pattern_instance.port_groups[pose_port_group_.port_group_index];
        for (const auto& port_group_instance : pose_groups) {
            int marker_id;
            port_group_instance->setValueFromParameter("marker_id", marker_id);
            marker_id_to_port_group_.emplace(marker_id, port_group_instance->port_group_instance_index);
        }

        return true;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input_image = data.getInput<InPortImage>().value();
        const auto &input_calibration = data.getInput<InPortCalibration>();

        std::vector<std::vector<cv::Point2f>> markers, rejected_candidates;
        std::vector<int32_t> marker_ids;
        cv::aruco::detectMarkers(
            input_image,
            dictionary_,
            markers,
            marker_ids,
            parameter_,
            rejected_candidates);

        if (debug_port_group_.size > 0) {
            // there can be only one of this port group
            SPDLOG_TRACE("create debug image");
            auto& output = data.getOutput<OutPortGroupDebug>(debug_port_group_.port_group_index, 0).value();
            auto& output_header = data.getOutputHeader<OutPortGroupDebug>(debug_port_group_.port_group_index, 0);
            output_header.copyFrom(data.getInputHeader<InPortImage>());
            output_header.pixel_format = PixelFormat::RGB;
            output_header.base_type = BaseType::UINT_8;
            output_header.channels = 3;
            output_header.stride = output_header.width;
            cv::cvtColor(input_image, output, cv::COLOR_GRAY2RGB);
            cv::aruco::drawDetectedMarkers(output, markers, marker_ids);
        }

        if (!marker_ids.empty()) {
            cv::Mat cameraMatrix;
            cv::Mat distortionCoefficientsMatrix;
            traact2cv(input_calibration, cameraMatrix, distortionCoefficientsMatrix);

            std::vector<cv::Vec3d> r_vecs;
            std::vector<cv::Vec3d> t_vecs;

            cv::aruco::estimatePoseSingleMarkers(
                markers, marker_size_,
                cameraMatrix,
                distortionCoefficientsMatrix,
                r_vecs,
                t_vecs);

            for (size_t marker_index = 0; marker_index < marker_ids.size(); ++marker_index) {
                auto marker_id = marker_ids[marker_index];
                auto is_output_marker = marker_id_to_port_group_.find(marker_id);
                if (is_output_marker == marker_id_to_port_group_.end()) {
                    continue;
                }
                auto port_group_instance_index = is_output_marker->second;
                SPDLOG_TRACE("send pose for port group instance {0}", port_group_instance_index);
                auto& output = data.getOutput<OutPortGroupPose>(pose_port_group_.port_group_index, port_group_instance_index );
                cv2traact(r_vecs[marker_index], t_vecs[marker_index], output);
            }
        }

        return true;
    }

 private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameter_;
    double marker_size_;
    std::map<int, int> marker_id_to_port_group_;
    PortGroupInfo pose_port_group_;
    PortGroupInfo debug_port_group_;

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvArucoTracker)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvArucoTracker)
END_TRAACT_PLUGIN_REGISTRATION
