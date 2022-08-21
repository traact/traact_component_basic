/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/aruco.hpp>
#include <traact/opencv/OpenCVUtils.h>

#include <apriltag.h>
#include <tagStandard41h12.h>
#include <apriltag_pose.h>

namespace traact::component::opencv {

class ApriltagTracker : public Component {
 public:
    using InPortImage = buffer::PortConfig<traact::vision::ImageHeader, 0>;
    using InPortCalibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;

    using OutPortGroupPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    //using OutPortGroupDebug = buffer::PortConfig<traact::vision::ImageHeader, 0>;

    explicit ApriltagTracker(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("ApriltagTracker",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortCalibration>("input_calibration");

        pattern->beginPortGroup("output_pose", 1)
            .addProducerPort<OutPortGroupPose>("output")
            .addParameter("marker_id", 0, 0, 10)
            .addParameter("marker_size", 0.08, 0.0, 1.0)
            .endPortGroup();
//        pattern->beginPortGroup("output_debug", 0, 1)
//            .addProducerPort<OutPortGroupDebug>("output")
//            .endPortGroup();

        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        pose_port_group_ = pattern_instance.getPortGroupInfo("output_pose");
        //debug_port_group_ = pattern_instance.getPortGroupInfo("output_debug");

    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override {

        const auto& pose_groups = pattern_instance.port_groups[pose_port_group_.port_group_index];
        for (const auto& port_group_instance : pose_groups) {
            int marker_id;
            double marker_size;
            port_group_instance->setValueFromParameter("marker_id", marker_id);
            port_group_instance->setValueFromParameter("marker_size", marker_size);
            marker_id_to_port_group_.emplace(marker_id, port_group_instance->port_group_instance_index);
            marker_id_to_size_group_.emplace(marker_id, marker_size);
        }

        apriltag_family_ = tagStandard41h12_create();
        apriltag_detector_ = apriltag_detector_create();
        apriltag_detector_add_family_bits(apriltag_detector_, apriltag_family_, 1);
        apriltag_detector_->quad_decimate = 1.f;
        apriltag_detector_->quad_sigma = 0.8f;
        apriltag_detector_->nthreads = 8;
        apriltag_detector_->refine_edges = 1;
        apriltag_detector_->decode_sharpening = 0.25;

        return true;
    }

    void destroyAprilTag() {

        if(apriltag_detector_){
            apriltag_detector_destroy(apriltag_detector_);
        }
        if(apriltag_family_){
            tagStandard41h12_destroy(apriltag_family_);
        }

    }
    bool teardown() override {
        destroyAprilTag();
        return true;
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input_image = data.getInput<InPortImage>().value();
        const auto &input_calibration = data.getInput<InPortCalibration>();

        image_u8_t april_image { .width = input_image.cols,
            .height = input_image.rows,
            .stride = static_cast<int>(input_image.step),
            .buf = input_image.data
        };



        zarray_t* detections = apriltag_detector_detect(apriltag_detector_, &april_image);

        apriltag_detection_info_t info;
        
        info.cx = input_calibration.cx;
        info.cy = input_calibration.cy;
        info.fx = input_calibration.fx;
        info.fy = input_calibration.fy;

        SPDLOG_DEBUG("april tag detected markers {0}", zarray_size(detections));


        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            SPDLOG_DEBUG("april tag found marker {0}", det->id);

            auto port_index = marker_id_to_port_group_.find(det->id);
            if(port_index == marker_id_to_port_group_.end()){
                continue;
            }

            info.det = det;
            info.tagsize = marker_id_to_size_group_[det->id];
            apriltag_pose_t april_pose;
            double err = estimate_tag_pose(&info, &april_pose);

            SPDLOG_TRACE("april tag estimated marker {0} error {1}", det->id, err);


            Eigen::Map<Eigen::Matrix3d> rotation(&april_pose.R->data[0]);
            Eigen::Map<Eigen::Vector3d> translation(&april_pose.t->data[0]);

            Eigen::Affine3d pose = Eigen::Translation3d(translation) * Eigen::Quaterniond(rotation.transpose());

            data.getOutput<OutPortGroupPose>(pose_port_group_.port_group_index, port_index->second) = pose.cast<Scalar>();


        }

        apriltag_detections_destroy(detections);




        return true;
    }

 private:
    PortGroupInfo pose_port_group_;
    std::map<int, int> marker_id_to_port_group_;
    std::map<int, double> marker_id_to_size_group_;
    apriltag_family_t* apriltag_family_{nullptr};
    apriltag_detector_t* apriltag_detector_{nullptr};

};

CREATE_TRAACT_COMPONENT_FACTORY(ApriltagTracker)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::ApriltagTracker)
END_TRAACT_PLUGIN_REGISTRATION
