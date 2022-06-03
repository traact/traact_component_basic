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
#include <traact/spatial.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/UndistortionHelper.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <traact/opencv/OpenCVUtils.h>


namespace traact::component::vision {

    class OpenCVArucoTracker : public Component {
    public:
        explicit OpenCVArucoTracker(const std::string &name) : Component(name,
                                                                           traact::component::ComponentType::SyncFunctional) {
        }

        traact::pattern::Pattern::Ptr GetPattern()  const {


            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("OpenCVArucoTracker", unlimited);

            pattern->addConsumerPort("input", traact::vision::ImageHeader::MetaType);
            pattern->addConsumerPort("input_calibration", traact::vision::CameraCalibrationHeader::MetaType);
            pattern->addProducerPort("output_ID1", traact::spatial::Pose6DHeader::MetaType);
            pattern->addProducerPort("output_ID2", traact::spatial::Pose6DHeader::MetaType);
            pattern->addProducerPort("output_ID3", traact::spatial::Pose6DHeader::MetaType);
            pattern->addProducerPort("output_ID4", traact::spatial::Pose6DHeader::MetaType);
            pattern->addProducerPort("output_ID5", traact::spatial::Pose6DHeader::MetaType);
            pattern->addProducerPort("output_ID6", traact::spatial::Pose6DHeader::MetaType);

            pattern->addCoordinateSystem("ImagePlane")
                    .addCoordinateSystem("Image", true)
                    .addCoordinateSystem("Camera")
                    .addCoordinateSystem("MarkerID1")
                    .addCoordinateSystem("MarkerID2")
                    .addCoordinateSystem("MarkerID3")
                    .addCoordinateSystem("MarkerID4")
                    .addCoordinateSystem("MarkerID5")
                    .addCoordinateSystem("MarkerID6")
                    .addEdge("ImagePlane", "Image", "input")
                    .addEdge("ImagePlane", "Image", "input_calibration")
                    .addEdge("Camera", "MarkerID1", "output_ID1")
                    .addEdge("Camera", "MarkerID2", "output_ID2")
                    .addEdge("Camera", "MarkerID3", "output_ID3")
                    .addEdge("Camera", "MarkerID4", "output_ID4")
                    .addEdge("Camera", "MarkerID5", "output_ID5")
                    .addEdge("Camera", "MarkerID6", "output_ID6");


            pattern->addParameter("markerSize", 0.181);

            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
            pattern::setValueFromParameter(parameter, "markerSize", markerSize_, 0.1);
            return true;
        }

        bool processTimePoint(buffer::ComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& image = data.getInput<ImageHeader::NativeType, ImageHeader>(0);
            const auto& calibration = data.getInput<CameraCalibrationHeader::NativeType, CameraCalibrationHeader>(1);

            const auto& cv_image = image.GetCpuMat();
            cv::Mat gray_image;
            switch (cv_image.type()) {

                case CV_8UC1:
                    gray_image = cv_image;
                    break;
                case CV_8UC3:
                    cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
                    break;
                case CV_8UC4:
                    cv::cvtColor(cv_image, gray_image, cv::COLOR_BGRA2GRAY);
                    break;
                default:
                    spdlog::error("unsupported image format for aruco tracker");
                    return false;

            }





            auto dictionary_ =  cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
            auto detectorParams_ = cv::aruco::DetectorParameters::create();

            std::vector<std::vector<cv::Point2f>> markers, rejectedCandidates;
            std::vector<int32_t> markerIds;
            cv::aruco::detectMarkers(
                    gray_image,
                    dictionary_,
                    markers,
                    markerIds,
                    detectorParams_,
                    rejectedCandidates);

            if (!markerIds.empty())
            {
                cv::Mat cameraMatrix;
                cv::Mat distortionCoefficientsMatrix;
                traact2cv(calibration, cameraMatrix, distortionCoefficientsMatrix);

                // Vectors for pose (translation and rotation) estimation
                std::vector<cv::Vec3d> rVecs;
                std::vector<cv::Vec3d> tVecs;

                // Estimate pose of single markers
                cv::aruco::estimatePoseSingleMarkers(
                        markers,
                        markerSize_,
                        cameraMatrix,
                        distortionCoefficientsMatrix,
                        rVecs,
                        tVecs);




                for (size_t i = 0; i < markerIds.size(); i++)
                {
                    int32_t  markerID = markerIds[i];
                    if(1 <  markerID && markerID > 6)
                        continue;

                    auto& result = data.getOutput<spatial::Pose6DHeader::NativeType, spatial::Pose6DHeader>(i);
                    cv2traact(rVecs[i],tVecs[i],result);
                }
            }

            return true;
        }


    private:
        float markerSize_{0.1};

    RTTR_ENABLE(Component)

    };

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::OpenCVArucoTracker>("OpenCVArucoTracker").constructor<std::string>()();
}