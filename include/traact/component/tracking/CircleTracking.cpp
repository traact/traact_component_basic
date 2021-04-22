/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

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
                                                                         traact::component::ComponentType::Functional) {
        }

        traact::pattern::Pattern::Ptr GetPattern()  const {


            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("CircleTracking", unlimited);

            pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
            //pattern->addConsumerPort("input_16Bit", traact::vision::ImageHeader::MetaType);
            pattern->addProducerPort("output", traact::spatial::Position2DListHeader::MetaType);


            return pattern;
        }



        bool processTimePoint(buffer::ComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& image = data.getInput<ImageHeader::NativeType, ImageHeader>(0);
            const auto& image_cpu = image.GetCpuMat();
            //const auto& image_16 = data.getInput<ImageHeader::NativeType, ImageHeader>(1);
            //const auto& image_cpu_16 = image_16.GetCpuMat();
            auto& output = data.getOutput<traact::spatial::Position2DListHeader::NativeType, traact::spatial::Position2DListHeader>(0);
            output.clear();

            //cv::Mat gray;

            //image.GetCpuMat().convertTo(gray, CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), 255. / 6000.0);

            cv::SimpleBlobDetector::Params params;

            params.filterByInertia = false;
            params.filterByConvexity = false;
            params.filterByColor = false;
            params.filterByCircularity = true;
            params.filterByArea = true;

            params.minDistBetweenBlobs = 0.0f;

            params.minThreshold = 180;
            params.maxThreshold = 255;



            params.minArea = 2*2;
            float maxAreaFactor = 0.1;
            float maxArea = maxAreaFactor*image_cpu.cols;
            params.maxArea = maxArea*maxArea;

            params.maxInertiaRatio = 1;
            params.minInertiaRatio = 0.5;

            params.minCircularity = 0.785;
            params.maxCircularity = 1;


            const int maxBlobDetectionPixelError = 0;
            const unsigned char threshold = params.minThreshold;
            const int maxRadius = image_cpu.cols / 16;



            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
            std::vector<cv::KeyPoint> keypoints;
            detector->detect( image_cpu, keypoints);

//            for(auto circle : keypoints) {
//
//                spdlog::info("found point {0} : {1}", circle.pt.x, circle.pt.y);
//                output.push_back(Eigen::Vector2d(circle.pt.x,circle.pt.y));
//            }

            detectCirclesMethod2(keypoints, reinterpret_cast<unsigned char*>(image_cpu.data), image_cpu.cols, maxBlobDetectionPixelError, threshold, maxRadius, output);

            if(keypoints.size() == output.size())
            for(int i=0;i<keypoints.size();++i){

                auto circle = keypoints[i];
                auto point = output[i];

                //spdlog::trace("blob found point {0} : {1}", circle.pt.x, circle.pt.y);
                //spdlog::trace("circle found point {0} : {1}", point.x(), point.y());


            }

            return true;
        }


    private:

        template<typename T>
        void detectCirclesMethod2 (const std::vector<cv::KeyPoint>& keypoints,T* imageData, int rowSize, const int maxBlobDetectionPixelError, const T threshold,const int maxRadius, spatial::Position2DList & result ) {


            traact::vision::ThresholdSubPixelEdgeDectection<T> edgeDetection;

            for(auto circle : keypoints) {
                int cx = std::round(circle.pt.x);
                int cy = std::round(circle.pt.y);


                T* startPixel = imageData + rowSize*cy + cx;



                //TODO check maxRadius for image borders
                int maxXPos = std::round(cx + edgeDetection.findEdge(startPixel, 1, threshold,maxRadius))+maxBlobDetectionPixelError;
                int minXPos = std::round(cx - edgeDetection.findEdge(startPixel, -1, threshold,maxRadius))-maxBlobDetectionPixelError;

                int maxYPos = std::round(cy + edgeDetection.findEdge(startPixel, rowSize, threshold,maxRadius))+maxBlobDetectionPixelError;
                int minYPos = std::round(cy - edgeDetection.findEdge(startPixel, -rowSize, threshold,maxRadius))-maxBlobDetectionPixelError;

                std::vector< cv::Point2f > subpixelPoints;
                std::vector< cv::Point2d > xSubpixelPoints;
                std::vector< double > xSubpixelCenter;
                for(int yIndex = minYPos; yIndex <= maxYPos; ++yIndex){

                    T* xStartPixel = imageData + rowSize*yIndex + cx;

                    double xRight = cx + edgeDetection.findEdge(xStartPixel, 1, threshold, maxRadius);
                    double xLeft = cx - edgeDetection.findEdge(xStartPixel, - 1, threshold, maxRadius);

                    if(xRight != static_cast<double>(cx) && xLeft != static_cast<double>(cx)) {
                        xSubpixelPoints.push_back(cv::Point2d(xRight, yIndex));
                        xSubpixelPoints.push_back(cv::Point2d(xLeft, yIndex));
                        subpixelPoints.push_back(cv::Point2f(xRight, yIndex));
                        subpixelPoints.push_back(cv::Point2f(xLeft, yIndex));
                        xSubpixelCenter.push_back((xRight+xLeft)/2.0);
                    }



                }

                std::vector< cv::Point2d > ySubpixelPoints;
                std::vector< double > ySubpixelCenter;
                for(int xIndex = minXPos; xIndex <= maxXPos; ++xIndex){

                    T* yStartPixel = imageData + rowSize*cy + xIndex;

                    double yTop = cy + edgeDetection.findEdge(yStartPixel, rowSize, threshold, maxRadius);
                    double yBottom = cy - edgeDetection.findEdge(yStartPixel, -rowSize, threshold, maxRadius);

                    if(yTop != static_cast<double>(cy) && yBottom != static_cast<double>(cy)) {
                        ySubpixelPoints.push_back(cv::Point2d(xIndex, yTop));
                        ySubpixelPoints.push_back(cv::Point2d(xIndex, yBottom));
                        subpixelPoints.push_back(cv::Point2f(xIndex, yTop));
                        subpixelPoints.push_back(cv::Point2f(xIndex, yBottom));
                        ySubpixelCenter.push_back((yTop+yBottom)/2.0);
                    }



                }

                if(xSubpixelPoints.size() > 0 && ySubpixelPoints.size() > 0) {
                    cv::Vec4d topToBottomLine;
                    cv::fitLine( xSubpixelPoints, topToBottomLine, cv::DIST_L2, 0, 0.01, 0.01 );

                    cv::Vec4d leftToRightLine;
                    cv::fitLine( ySubpixelPoints, leftToRightLine, cv::DIST_L2, 0, 0.01, 0.01 );

                    const double x1 = topToBottomLine[2];
                    const double y1 = topToBottomLine[3];
                    const double x2 = x1 + topToBottomLine[0];
                    const double y2 = y1 + topToBottomLine[1];

                    const double x3 = leftToRightLine[2];
                    const double y3 = leftToRightLine[3];
                    const double x4 = x3 + leftToRightLine[0];
                    const double y4 = y3 + leftToRightLine[1];

                    const double tmp = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
                    double subX = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/tmp;
                    double subY = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/tmp;

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

                    //spdlog::info("found point {0} : {1}", subX, subY);
                    result.push_back(Eigen::Vector2d(subX,subY));

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