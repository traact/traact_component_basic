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
#include <traact/opencv/OpenCVUtils.h>
namespace traact::component {

    class OpenCVUndistort2DPoints : public Component {
    public:
        explicit OpenCVUndistort2DPoints(const std::string &name) : Component(name,
                                                                         traact::component::ComponentType::SyncFunctional) {
        }

        traact::pattern::Pattern::Ptr GetPattern()  const {


            traact::pattern::Pattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::Pattern>("OpenCVUndistort2DPoints", serial);

            pattern->addConsumerPort("input", traact::spatial::Position2DListHeader::MetaType);
            pattern->addConsumerPort("input_calibration", traact::vision::CameraCalibrationHeader::MetaType);
            pattern->addProducerPort("output", traact::spatial::Position2DListHeader::MetaType);

            pattern->addCoordinateSystem("ImagePlane_Distorted")
            .addCoordinateSystem("ImagePlane_Undistorted")
            .addCoordinateSystem("Points")
             .addEdge("ImagePlane_Distorted","ImagePlane_Undistorted","input_calibration")
             .addEdge("ImagePlane_Distorted","Points","input")
             .addEdge("ImagePlane_Undistorted","Points","output");


            return pattern;
        }


        bool processTimePoint(buffer::ComponentBuffer &data) override {
            using namespace traact::spatial;
            using namespace traact::vision;
            const auto& input = data.getInput<Position2DListHeader::NativeType, Position2DListHeader>(0);
            const auto& calibration = data.getInput<CameraCalibrationHeader::NativeType, CameraCalibrationHeader>(1);
            auto& output = data.getOutput<Position2DListHeader::NativeType, Position2DListHeader>(0);

            output.resize(input.size());
            if(input.empty())
                return true;

            cv::Mat cv_intrinsics;
            cv::Mat cv_distortion;
            traact2cv(calibration,cv_intrinsics, cv_distortion);
            std::vector<cv::Point2f> opencv_points;

            for(auto& point : input) {
                opencv_points.push_back(eigen2cv(point));
            }
            std::vector<cv::Point2f> opencv_points_output(opencv_points.size());
            cv::undistortPoints(opencv_points,opencv_points_output, cv_intrinsics, cv_distortion, cv::noArray(), cv_intrinsics);

            for(int i=0;i<opencv_points_output.size();++i){
                output[i].x() = opencv_points_output[i].x;
                output[i].y() = opencv_points_output[i].y;
            }


            return true;
        }


    private:
        double alpha_;
        double beta_;

    RTTR_ENABLE(Component)

    };

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::OpenCVUndistort2DPoints>("OpenCVUndistort2DPoints").constructor<std::string>()();
}