/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_SPATIAL_MODULE_SRC_TRAACT_COMPONENT_SPATIAL_UTIL_POSE6DTESTSOURCE_H_
#define TRAACT_SPATIAL_MODULE_SRC_TRAACT_COMPONENT_SPATIAL_UTIL_POSE6DTESTSOURCE_H_

#include <traact/traact.h>
#include <traact/spatial.h>
#include <thread>
#include <traact/buffer/SourceComponentBuffer.h>

namespace traact::component::util {
class Pose6DTestSource : public Component {
 public:
    explicit Pose6DTestSource(const std::string &name) : Component(name) {
        internal_data_.setIdentity();
        running_ = false;
    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::spatial;

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("Pose6DTestSource", Concurrency::UNLIMITED, ComponentType::SYNC_SOURCE);

        pattern->addProducerPort("output", Pose6DHeader::MetaType);
        pattern->addCoordinateSystem("Origin", false)
            .addCoordinateSystem("Target", false)
            .addEdge("Origin", "Target", "output");

        return pattern;
    }

    bool start() override {
        running_ = true;
        SPDLOG_INFO("starting simple source");
        thread_.reset(new std::thread(std::bind(&Pose6DTestSource::threadLoop, this)));
        return true;
    }
    bool stop() override {
        SPDLOG_INFO("stopping simple source");
        if (running_) {
            running_ = false;
            thread_->join();
        }
        return true;
    }

    void waitForFinish() {
        thread_->join();
    }

 private:
    Eigen::Affine3d internal_data_;
    std::shared_ptr<std::thread> thread_;
    bool running_;

    void threadLoop() {
        using namespace traact::spatial;
        using namespace traact;
        Timestamp ts = traact::kTimestampZero;
        //Timestamp ts = now();
        TimeDuration deltaTs = std::chrono::milliseconds(10);

        int output_count = 0;
        internal_data_ = internal_data_ * Eigen::Translation3d(0, 0, 1);

        while (running_ && output_count < 1000) {
            std::this_thread::sleep_for(deltaTs);
            ts += std::chrono::milliseconds(10);

            SPDLOG_TRACE("request buffer");

            auto buffer_future = request_callback_(ts);
            buffer_future.wait();
            auto buffer = buffer_future.get();
            if (buffer == nullptr) {
                SPDLOG_WARN("request for buffer at ts {0} was rejected", ts.time_since_epoch().count());
                continue;
            }

            auto &newData = buffer->getOutput<Pose6DHeader::NativeType, Pose6DHeader>(0);

            newData = internal_data_;

            SPDLOG_TRACE("commit data");
            buffer->commit(true);

            internal_data_ = internal_data_ * Eigen::Translation3d(0, 0, 1);
            output_count++;

            SPDLOG_TRACE("done");
        }
        SPDLOG_TRACE("source quit loop");
        running_ = false;
    }

};

}

#endif //TRAACT_SPATIAL_MODULE_SRC_TRAACT_COMPONENT_SPATIAL_UTIL_POSE6DTESTSOURCE_H_
