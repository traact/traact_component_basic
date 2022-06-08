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

namespace traact::component {

class CircleTracking : public Component {
 public:
    explicit CircleTracking(const std::string &name) : Component(name,
                                                                 traact::component::ComponentType::SYNC_FUNCTIONAL) {
    }

    traact::pattern::Pattern::Ptr GetPattern() const {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("CircleTracking", Concurrency::UNLIMITED);

        pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
        //pattern->addConsumerPort("input_16Bit", traact::vision::ImageHeader::MetaType);
        pattern->addProducerPort("output", traact::spatial::Position2DListHeader::MetaType);
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

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(parameter, "threshold", threshold, threshold);
        pattern::setValueFromParameter(parameter, "filter_area", filter_area_, filter_area_);
        if (filter_area_) {
            pattern::setValueFromParameter(parameter, "area_min", area_min_, area_min_);
            pattern::setValueFromParameter(parameter, "area_max", area_max_, area_max_);
        }
        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<ImageHeader>(0);
        const auto &image_cpu = image.GetCpuMat();
        //const auto& image_16 = data.getInput<ImageHeader::NativeType, ImageHeader>(1);
        //const auto& image_cpu_16 = image_16.GetCpuMat();
        auto &output = data.getOutput<traact::spatial::Position2DListHeader>(0);
        output.clear();

        //cv::Mat gray;

        //image.GetCpuMat().convertTo(gray, CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), 255. / 6000.0);

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
        const int maxRadius = image_cpu.cols / 16;

        try {
            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
            std::vector<cv::KeyPoint> keypoints;
            detector->detect(image_cpu, keypoints);

//            for(auto circle : keypoints) {
//
//                SPDLOG_INFO("found point {0} : {1}", circle.pt.x, circle.pt.y);
//                output.push_back(Eigen::Vector2d(circle.pt.x,circle.pt.y));
//            }

            detectCirclesMethod2(keypoints,
                                 reinterpret_cast<unsigned char *>(image_cpu.data),
                                 image_cpu.cols,
                                 maxBlobDetectionPixelError,
                                 threshold,
                                 maxRadius,
                                 output);

            if (keypoints.size() == output.size())
                for (int i = 0; i < keypoints.size(); ++i) {

                    auto circle = keypoints[i];
                    auto point = output[i];

//                        point.x() = circle.pt.x;
//                        point.y() = circle.pt.y;

                    //SPDLOG_TRACE("blob found point {0} : {1}", circle.pt.x, circle.pt.y);
                    //SPDLOG_TRACE("circle found point {0} : {1}", point.x(), point.y());


                }
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

    template<typename T>
    void detectCirclesMethod2(const std::vector<cv::KeyPoint> &keypoints,
                              T *imageData,
                              int rowSize,
                              const int maxBlobDetectionPixelError,
                              const T threshold,
                              const int maxRadius,
                              spatial::Position2DList &result) {

        traact::vision::ThresholdSubPixelEdgeDectection<T> edgeDetection;

        for (auto circle : keypoints) {
            int cx = std::round(circle.pt.x);
            int cy = std::round(circle.pt.y);

            T *startPixel = imageData + rowSize * cy + cx;



            //TODO check maxRadius for image borders
            int maxXPos = std::round(cx + edgeDetection.findEdge(startPixel, 1, threshold, maxRadius))
                + maxBlobDetectionPixelError;
            int minXPos = std::round(cx - edgeDetection.findEdge(startPixel, -1, threshold, maxRadius))
                - maxBlobDetectionPixelError;

            int maxYPos = std::round(cy + edgeDetection.findEdge(startPixel, rowSize, threshold, maxRadius))
                + maxBlobDetectionPixelError;
            int minYPos = std::round(cy - edgeDetection.findEdge(startPixel, -rowSize, threshold, maxRadius))
                - maxBlobDetectionPixelError;

            std::vector<cv::Point2f> subpixelPoints;
            std::vector<cv::Point2d> xSubpixelPoints;
            std::vector<double> xSubpixelCenter;
            for (int yIndex = minYPos; yIndex <= maxYPos; ++yIndex) {

                T *xStartPixel = imageData + rowSize * yIndex + cx;

                double xRight = cx + edgeDetection.findEdge(xStartPixel, 1, threshold, maxRadius);
                double xLeft = cx - edgeDetection.findEdge(xStartPixel, -1, threshold, maxRadius);

                if (xRight != static_cast<double>(cx) && xLeft != static_cast<double>(cx)) {
                    xSubpixelPoints.push_back(cv::Point2d(xRight, yIndex));
                    xSubpixelPoints.push_back(cv::Point2d(xLeft, yIndex));
                    subpixelPoints.push_back(cv::Point2f(xRight, yIndex));
                    subpixelPoints.push_back(cv::Point2f(xLeft, yIndex));
                    xSubpixelCenter.push_back((xRight + xLeft) / 2.0);
                }

            }

            std::vector<cv::Point2d> ySubpixelPoints;
            std::vector<double> ySubpixelCenter;
            for (int xIndex = minXPos; xIndex <= maxXPos; ++xIndex) {

                T *yStartPixel = imageData + rowSize * cy + xIndex;

                double yTop = cy + edgeDetection.findEdge(yStartPixel, rowSize, threshold, maxRadius);
                double yBottom = cy - edgeDetection.findEdge(yStartPixel, -rowSize, threshold, maxRadius);

                if (yTop != static_cast<double>(cy) && yBottom != static_cast<double>(cy)) {
                    ySubpixelPoints.push_back(cv::Point2d(xIndex, yTop));
                    ySubpixelPoints.push_back(cv::Point2d(xIndex, yBottom));
                    subpixelPoints.push_back(cv::Point2f(xIndex, yTop));
                    subpixelPoints.push_back(cv::Point2f(xIndex, yBottom));
                    ySubpixelCenter.push_back((yTop + yBottom) / 2.0);
                }

            }

            if (!xSubpixelPoints.empty() && !ySubpixelPoints.empty() && subpixelPoints.size() > 5) {
                cv::Vec4d topToBottomLine;
                cv::fitLine(xSubpixelPoints, topToBottomLine, cv::DIST_L2, 0, 0.01, 0.01);

                cv::Vec4d leftToRightLine;
                cv::fitLine(ySubpixelPoints, leftToRightLine, cv::DIST_L2, 0, 0.01, 0.01);

                const double x1 = topToBottomLine[2];
                const double y1 = topToBottomLine[3];
                const double x2 = x1 + topToBottomLine[0];
                const double y2 = y1 + topToBottomLine[1];

                const double x3 = leftToRightLine[2];
                const double y3 = leftToRightLine[3];
                const double x4 = x3 + leftToRightLine[0];
                const double y4 = y3 + leftToRightLine[1];

                const double tmp = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
                double subX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / tmp;
                double subY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / tmp;

                //SPDLOG_TRACE("elipse points {0}", subpixelPoints.size());
                cv::RotatedRect ellipse = cv::fitEllipse(subpixelPoints);
                subX = ellipse.center.x;
                subY = ellipse.center.y;


//                    double errorX = 0;
//                    for (const auto &center : xSubpixelCenter) {
//                        const double tmp = center - subX;
//                        errorX += tmp * tmp;
//                    }
//                    errorX = errorX / xSubpixelCenter.size();
//
//                    double errorY = 0;
//                    for (const auto &center : ySubpixelCenter) {
//                        const double tmp = center - subY;
//                        errorY += tmp * tmp;
//                    }
//                    errorY = errorY / ySubpixelCenter.size();

                //SPDLOG_INFO("found point {0} : {1}", subX, subY);
                result.push_back(Eigen::Vector2d(subX, subY));

            }
        }

    }

 RTTR_ENABLE(Component)

};

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::CircleTracking>("CircleTracking").constructor<std::string>()();
}