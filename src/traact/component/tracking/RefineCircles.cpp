/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/SubPixelEdgeDectection.h>
#include <traact/spatial.h>

namespace traact::component::tracking {

class RefineCircles : public Component {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortPoints = buffer::PortConfig<vision::KeyPointListHeader, 1>;
    using OutPortPoints = buffer::PortConfig<vision::KeyPointListHeader, 0>;
    explicit RefineCircles(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RefineCircles",
                                                       Concurrency::UNLIMITED,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortImage>("input")
            .addConsumerPort<InPortPoints>("input_points")
            .addProducerPort<OutPortPoints>("output")
            .addParameter("Threshold", 160, 0, 255)
            .addParameter("MaxRadius", 32);

        pattern->addCoordinateSystem("Camera").addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Points")
            .addEdge("Camera", "ImagePlane", "input")
            .addEdge("ImagePlane", "Points", "output");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("Threshold", threshold_);
        pattern_instance.setValueFromParameter("MaxRadius", max_radius_);
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<InPortImage>().getImage();
        const auto &points = data.getInput<InPortPoints>();

        auto &output = data.getOutput<OutPortPoints>();
        output.clear();

        try {

            detectCirclesMethod2(points,
                                 reinterpret_cast<unsigned char *>(image.data),
                                 image.cols,
                                 image.cols*image.rows,
                                 output);

        } catch (...) {
            return false;
        }

        return true;
    }

 private:

    int threshold_{160};
    float max_radius_;
    bool filter_area_{false};
    double area_min_{1};
    double area_max_{std::numeric_limits<double>::max()};

    template<typename T>
    void detectCirclesMethod2(const std::vector<cv::KeyPoint> &keypoints,
                              T *image_data,
                              int width,
                              size_t image_size,
                              std::vector<cv::KeyPoint> &result) {

        traact::vision::ThresholdSubPixelEdgeDectection<T> edgeDetection;

        result = keypoints;

        for (size_t i = 0; i < keypoints.size(); ++i) {
            const auto &circle = keypoints[i];
            int cx = std::round(circle.pt.x);
            int cy = std::round(circle.pt.y);

            T *startPixel = image_data + width * cy + cx;


            int max_x_pos = std::round(cx + edgeDetection.findEdge(startPixel,image_size, 1, threshold_, max_radius_));
            int min_x_pos = std::round(cx - edgeDetection.findEdge(startPixel,image_size, -1, threshold_, max_radius_));

            int max_y_pos = std::round(cy + edgeDetection.findEdge(startPixel,image_size, width, threshold_, max_radius_));
            int min_y_pos = std::round(cy - edgeDetection.findEdge(startPixel,image_size, -width, threshold_, max_radius_));

            std::vector<cv::Point2f> subpixel_points;
            subpixel_points.reserve((max_y_pos-min_y_pos + max_x_pos-min_x_pos)*2);
            for (int y_index = min_y_pos; y_index <= max_y_pos; ++y_index) {

                T *x_start_pixel = image_data + width * y_index + cx;

                float x_right = cx + edgeDetection.findEdge(x_start_pixel,image_size, 1, threshold_, max_radius_);
                float x_left = cx - edgeDetection.findEdge(x_start_pixel,image_size, -1, threshold_, max_radius_);

                if (x_right != static_cast<float>(cx) && x_left != static_cast<float>(cx)) {
                    subpixel_points.template emplace_back(cv::Point2f(x_right, y_index));
                    subpixel_points.template emplace_back(cv::Point2f(x_left, y_index));
                }

            }

            for (int x_index = min_x_pos; x_index <= max_x_pos; ++x_index) {

                T *y_start_pixel = image_data + width * cy + x_index;

                float y_top = cy + edgeDetection.findEdge(y_start_pixel,image_size, width, threshold_, max_radius_);
                float y_bottom = cy - edgeDetection.findEdge(y_start_pixel,image_size, -width, threshold_, max_radius_);

                if (y_top != static_cast<float>(cy) && y_bottom != static_cast<float>(cy)) {
                    subpixel_points.template emplace_back(cv::Point2f(x_index, y_top));
                    subpixel_points.template emplace_back(cv::Point2f(x_index, y_bottom));
                }

            }

            if (subpixel_points.size() > 5) {
                cv::RotatedRect ellipse = cv::fitEllipse(subpixel_points);
                result[i].pt = ellipse.center;
                result[i].size = (ellipse.size.height+ellipse.size.width)/2.0F;
            }
        }

    }

};

CREATE_TRAACT_COMPONENT_FACTORY(RefineCircles)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::tracking::RefineCircles)
END_TRAACT_PLUGIN_REGISTRATION

