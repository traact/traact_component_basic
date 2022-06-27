/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>

namespace traact::component::opencv {

class OpenCvBlobDetection : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortPoints = buffer::PortConfig<vision::KeyPointListHeader, 0>;
    using OutPortFeatures = buffer::PortConfig<vision::FeatureListHeader, 1>;

    explicit OpenCvBlobDetection(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvBlobDetection",
                                                       Concurrency::UNLIMITED,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addProducerPort<OutPortPoints>("output")
            .addProducerPort<OutPortFeatures>("output_feature")
            .addParameter("FilterByArea", false)
            .addParameter("FilterByCircularity", false)
            .addParameter("FilterByConvexity", false)
            .addParameter("FilterByInertia", false)
            .addParameter("FilterByColor", false)
            .addParameter("MaxArea", 100)
            .addParameter("MinArea", 0)
            .addParameter("MaxCircularity", 1.0, 0.0, 1.0)
            .addParameter("MinCircularity", 0.0, 0.0, 1.0)
            .addParameter("MaxConvexity", 1.0, 0.0, 1.0)
            .addParameter("MinConvexity", 0.0, 0.0, 1.0)
            .addParameter("MaxInertiaRatio", 1.0, 0.0, 1.0)
            .addParameter("MinInertiaRatio", 0.0, 0.0, 1.0)
            .addParameter("MaxThreshold", 1.0, 0.0, 1.0)
            .addParameter("MinThreshold", 0.0, 0.0)
            .addParameter("MinDistBetweenBlobs", 0.0, 0.0)
            .addParameter("BlobColor", 0, 0,255);

        pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Points")
            .addEdge("Camera", "ImagePlane", "input")
            .addEdge("ImagePlane", "Points", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("FilterByArea", blob_detector_config_.filterByArea);
        pattern_instance.setValueFromParameter("FilterByCircularity", blob_detector_config_.filterByCircularity);
        pattern_instance.setValueFromParameter("FilterByConvexity", blob_detector_config_.filterByConvexity);
        pattern_instance.setValueFromParameter("FilterByInertia", blob_detector_config_.filterByInertia);
        pattern_instance.setValueFromParameter("FilterByColor", blob_detector_config_.filterByColor);
        pattern_instance.setValueFromParameter("MaxArea", blob_detector_config_.maxArea);
        pattern_instance.setValueFromParameter("MinArea", blob_detector_config_.minArea);
        pattern_instance.setValueFromParameter("MaxCircularity", blob_detector_config_.maxCircularity);
        pattern_instance.setValueFromParameter("MinCircularity", blob_detector_config_.minCircularity);
        pattern_instance.setValueFromParameter("MaxConvexity", blob_detector_config_.maxConvexity);
        pattern_instance.setValueFromParameter("MinConvexity", blob_detector_config_.minConvexity);
        pattern_instance.setValueFromParameter("MaxInertiaRatio", blob_detector_config_.maxInertiaRatio);
        pattern_instance.setValueFromParameter("MinInertiaRatio", blob_detector_config_.minInertiaRatio);
        pattern_instance.setValueFromParameter("MaxThreshold", blob_detector_config_.maxThreshold);
        pattern_instance.setValueFromParameter("MinThreshold", blob_detector_config_.minThreshold);
        pattern_instance.setValueFromParameter("MinDistBetweenBlobs", blob_detector_config_.minDistBetweenBlobs);
        pattern_instance.setValueFromParameter("BlobColor", blob_detector_config_.blobColor);

        feature_id_ = vision::createFeatureId();

        use_features_ = pattern_instance.getOutputPortsConnected(kDefaultTimeDomain).at(OutPortFeatures::PortIdx);

        return true;
    }

    virtual bool start() override {
        detector_ = cv::SimpleBlobDetector::create(blob_detector_config_);
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<InPortImage>().getImage();

        auto &output = data.getOutput<OutPortPoints>();
        output.clear();

        try {

            detector_->detect(image, output);
            if(use_features_){
                auto& output_features = data.getOutput<OutPortFeatures>();
                output_features.createIds(output.size(), feature_id_);
            }

        } catch (...) {
            return false;
        }

        return true;
    }

 private:
    cv::SimpleBlobDetector::Params blob_detector_config_;
    vision::FeatureID feature_id_;
    cv::Ptr<cv::SimpleBlobDetector> detector_;
    bool use_features_{false};

};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvBlobDetection)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvBlobDetection)
END_TRAACT_PLUGIN_REGISTRATION

