/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>

namespace traact::component::opencv {

class OpenCvBlobDetection : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using OutPortPoints = buffer::PortConfig<vision::Position2DListHeader, 0>;

    explicit OpenCvBlobDetection(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("OpenCvBlobDetection", Concurrency::UNLIMITED,ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input");
        pattern->addProducerPort<OutPortPoints>("output");
        pattern->addParameter("threshold", 160, 0, 255)
            .addParameter("filter_area", false)
            .addParameter("area_min", 1.0, 1.0, std::numeric_limits<double>::max())
            .addParameter("area_max", std::numeric_limits<double>::max(), 1.0, std::numeric_limits<double>::max());

        pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Points")
            .addEdge("Camera", "ImagePlane", "input")
            .addEdge("ImagePlane", "Points", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(pattern_instance, "threshold", threshold, threshold);
        pattern::setValueFromParameter(pattern_instance, "filter_area", filter_area_, filter_area_);
        if (filter_area_) {
            pattern::setValueFromParameter(pattern_instance, "area_min", area_min_, area_min_);
            pattern::setValueFromParameter(pattern_instance, "area_max", area_max_, area_max_);
        }
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto& image = data.getInput<InPortImage>().getImage();

        auto &output = data.getOutput<OutPortPoints>();
        output.clear();

        cv::SimpleBlobDetector::Params params;

        params.filterByInertia = false;
        params.filterByConvexity = false;
        params.filterByColor = false;
        params.filterByCircularity = true;
        params.filterByArea = filter_area_;

        params.minDistBetweenBlobs = 0.0f;

        params.minThreshold = threshold;
        params.maxThreshold = 255;

        if (filter_area_) {
            params.minArea = area_min_;
            params.maxArea = area_max_;
        }

        params.maxInertiaRatio = 1;
        params.minInertiaRatio = 0.5;

        params.minCircularity = 0.785;
        params.maxCircularity = 1;

        const int maxBlobDetectionPixelError = 0;
        const unsigned char threshold = params.minThreshold;
        const int maxRadius = image.cols / 16;

        try {
            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
            std::vector<cv::KeyPoint> keypoints;
            detector->detect(image, keypoints);



        } catch (...) {
            return false;
        }

        return true;
    }

 private:

    int threshold{160};
    bool filter_area_{false};
    double area_min_{1};
    double area_max_{std::numeric_limits<double>::max()};





};

CREATE_TRAACT_COMPONENT_FACTORY(OpenCvBlobDetection)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
REGISTER_DEFAULT_COMPONENT(traact::component::opencv::OpenCvBlobDetection)
END_TRAACT_PLUGIN_REGISTRATION

