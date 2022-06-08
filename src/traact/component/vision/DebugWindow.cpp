/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/highgui.hpp>
#include "OpenCVModule.h"
#include <rttr/registration>
#include <opencv2/imgproc.hpp>
#include <traact/math/perspective.h>

namespace traact::component::vision {

class DebugWindow : public Component {
 public:
    DebugWindow(const std::string &name)
        : Component(name, ComponentType::SYNC_SINK), wait_next_frame_(1, 1) {}

    traact::pattern::Pattern::Ptr GetPattern() const {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("DebugWindow", Concurrency::SERIAL);

        pattern->addConsumerPort("input", ImageHeader::MetaType);
        pattern->addConsumerPort("input_intrinsics", CameraCalibrationHeader::MetaType);
        pattern->addConsumerPort("input_2d_tracking", spatial::Position2DListHeader::MetaType);
        pattern->addConsumerPort("target_pose", spatial::Pose6DHeader::MetaType);

        pattern->addParameter("WaitForNextFrame", false);

        pattern->addCoordinateSystem("Camera")
            .addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Target")
            .addCoordinateSystem("TargetPoints")
            .addEdge("Camera", "ImagePlane", "input")
            .addEdge("ImagePlane", "Camera", "input_intrinsics")
            .addEdge("Camera", "Target", "target_pose")
            .addEdge("ImagePlane", "TargetPoints", "input_2d_tracking");

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        pattern::setValueFromParameter(parameter, "WaitForNextFrame", wait_, false);
        return Component::configure(parameter, data);
    }

    bool start() override {
        running_ = true;
        thread_ = std::make_shared<std::thread>([this] {
            thread_loop();
        });
        return true;
    }

    bool stop() override {
        running_ = false;
        thread_->join();
        return true;
    }

    bool processTimePoint(traact::DefaultComponentBuffer &data) override {
        using namespace traact::vision;

        if (wait_)
            wait_next_frame_.wait();

        {

            mea_idx_ = 0;//data.GetMeaIdx();
            const auto &intrinsics_ = data.getInput<CameraCalibrationHeader>(1);
            const auto &tracking_list_ = data.getInput<spatial::Position2DListHeader>(2);
            const auto &pose_c2w = data.getInput<spatial::Pose6DHeader>(3);
            const auto &image_ = data.getInput<ImageHeader>(0);

            cv::cvtColor(image_.GetCpuMat(), image_copy_, cv::COLOR_GRAY2BGR);
            for (const auto &point : tracking_list_) {
                cv::circle(image_copy_, cv::Point2d(point.x(), point.y()), 4, cv::Scalar_(255, 0, 0), 1);
            }

            auto p0 = traact::math::reproject_point(pose_c2w, intrinsics_,
                                                    Eigen::Vector3d(0, 0, 0));
            auto px = traact::math::reproject_point(pose_c2w, intrinsics_,
                                                    Eigen::Vector3d(1, 0, 0));
            auto py = traact::math::reproject_point(pose_c2w, intrinsics_,
                                                    Eigen::Vector3d(0, 1, 0));
            auto pz = traact::math::reproject_point(pose_c2w, intrinsics_,
                                                    Eigen::Vector3d(0, 0, 1));
            cv::line(image_copy_, cv::Point2d(p0.x(), p0.y()), cv::Point2d(px.x(), px.y()),
                     cv::Scalar(0, 0, 255), 1);
            cv::line(image_copy_, cv::Point2d(p0.x(), p0.y()), cv::Point2d(py.x(), py.y()),
                     cv::Scalar(0, 255, 0), 1);
            cv::line(image_copy_, cv::Point2d(p0.x(), p0.y()), cv::Point2d(pz.x(), pz.y()),
                     cv::Scalar(255, 0, 0), 1);

        }
        auto ts = data.getTimestamp();
        std::chrono::duration<double> diff = ts - last_ts_;
        last_ts_ = ts;
        fps_ = fps_ * 0.6 + 0.4 * 1.0 / diff.count();
        return true;
    }

    void invalidTimePoint(Timestamp ts, size_t mea_idx) override {
        mea_idx_ = mea_idx;

        std::chrono::duration<double> diff = ts - last_ts_;
        last_ts_ = ts;
        fps_ = fps_ * 0.6 + 0.4 * 1.0 / diff.count();

    }

 private:
    std::shared_ptr<std::thread> thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> wait_{false};
    Semaphore wait_next_frame_;
    cv::Mat image_copy_;
    size_t mea_idx_;

    double fps_{0};
    Timestamp last_ts_{Timestamp::min()};



//        tbb::queuing_mutex data_lock_;

    void thread_loop() {
        cv::namedWindow(getName(), cv::WINDOW_KEEPRATIO);

        while (running_) {

            if (!image_copy_.empty()) {
                auto image_clone = image_copy_.clone();
                cv::putText(image_clone,
                            fmt::format("Evt idx: {0}", mea_idx_),
                            cv::Point2d(10, 30),
                            cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                            1,
                            cv::Scalar(200, 200, 250),
                            1,
                            cv::LINE_AA);
                cv::putText(image_clone,
                            fmt::format("FPS: {0:.2f}", fps_),
                            cv::Point2d(10, 60),
                            cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                            1,
                            cv::Scalar(200, 200, 250),
                            1,
                            cv::LINE_AA);

                cv::imshow(getName(), image_clone);

            }
            int key = cv::waitKey(5);

            switch (key) {
                case 'n': {
                    wait_next_frame_.notify();
                    break;

                }
                case 'p': {
                    wait_ = !wait_;
                    break;
                }

                default:break;
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
    registration::class_<traact::component::vision::DebugWindow>("DebugWindow").constructor<std::string>()();
}