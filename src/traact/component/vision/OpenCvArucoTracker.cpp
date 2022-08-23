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
            .addParameter("marker_size", 0.08)
            .endPortGroup();
        pattern->beginPortGroup("output_debug", 0, 1)
            .addProducerPort<OutPortGroupDebug>("output")
            .endPortGroup();

        pattern->addParameter("dictionary", "DICT_4X4_50", {"DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50"});
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
                                       "dictionary",
                                       dict,
                                       "DICT_4X4_50",
                                       {{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                                        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                                        {"DICT_6X6_50", cv::aruco::DICT_6X6_50}});

        dictionary_ = cv::aruco::getPredefinedDictionary(dict);
        parameter_ = cv::aruco::DetectorParameters::create();

        marker_id_to_index_.clear();

        const auto& pose_groups = pattern_instance.port_groups[pose_port_group_.port_group_index];

        marker_index_port_group_.resize(pose_groups.size());
        marker_index_marker_size_.resize(pose_groups.size());

        size_t marker_index{0};

        for (const auto& port_group_instance : pose_groups) {
            int marker_id{0};
            double marker_size{0};
            port_group_instance->setValueFromParameter("marker_id", marker_id);
            port_group_instance->setValueFromParameter("marker_size", marker_size);


            marker_id_to_index_.emplace(marker_id, marker_index);
            marker_index_port_group_[marker_index] = port_group_instance->port_group_instance_index;
            marker_index_marker_size_[marker_index] = marker_size;
            ++marker_index;
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

            std::vector<cv::Vec3d> opencv_rotations;
            std::vector<cv::Vec3d> opencv_tranlations;
            std::vector<std::vector<cv::Point2f>> current_marker(1);





            for (size_t found_marker_index = 0; found_marker_index < marker_ids.size(); ++found_marker_index) {
                auto marker_id = marker_ids[found_marker_index];
                auto is_output_marker = marker_id_to_index_.find(marker_id);
                if (is_output_marker == marker_id_to_index_.end()) {
                    continue;
                }
                auto marker_index = is_output_marker->second;
                auto port_group_instance_index = marker_index_port_group_[marker_index];
                auto marker_size = marker_index_marker_size_[marker_index];

                current_marker[0] = std::move(markers[found_marker_index]);
                opencv_rotations.clear();
                opencv_tranlations.clear();

                cv::aruco::estimatePoseSingleMarkers(
                    current_marker, marker_size,
                    cameraMatrix,
                    distortionCoefficientsMatrix,
                    opencv_rotations,
                    opencv_tranlations);

                SPDLOG_TRACE("send pose for port group instance {0}", port_group_instance_index);
                auto& output = data.getOutput<OutPortGroupPose>(pose_port_group_.port_group_index, port_group_instance_index );
                cv2traact(opencv_rotations[0], opencv_tranlations[0], output);
                output.translation() *= marker_size;
            }
        }

        return true;
    }

 private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameter_;
    std::map<int, int> marker_id_to_index_{};
    std::vector<int> marker_index_port_group_{};
    std::vector<double> marker_index_marker_size_{};
    PortGroupInfo pose_port_group_;
    PortGroupInfo debug_port_group_;

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvArucoTracker)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvArucoTracker)
END_TRAACT_PLUGIN_REGISTRATION
