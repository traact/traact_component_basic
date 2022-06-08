/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "OpenCVModule.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
bool traact::component::vision::OpenCVModule::init(traact::component::Module::ComponentPtr module_component) {
    return Module::init(module_component);
}
bool traact::component::vision::OpenCVModule::start(traact::component::Module::ComponentPtr module_component) {
//    tbb::queuing_mutex::scoped_lock lock(running_lock_);
    if (running_)
        return true;
    SPDLOG_DEBUG("Starting OpenCV Module");
    running_ = true;
    thread_ = std::make_shared<std::thread>([this] {
        thread_loop();
    });
    //init_lock_.Wait();

    return Module::start(module_component);
}
bool traact::component::vision::OpenCVModule::stop(traact::component::Module::ComponentPtr module_component) {
    // tbb::queuing_mutex::scoped_lock lock(running_lock_);
    if (!running_)
        return true;
    running_ = false;
    thread_->join();
    return Module::stop(module_component);
}
bool traact::component::vision::OpenCVModule::teardown(traact::component::Module::ComponentPtr module_component) {
    return Module::teardown(module_component);
}

void traact::component::vision::OpenCVModule::thread_loop() {

    for (const auto &window_component : window_components_) {
        cv::namedWindow(window_component.first, cv::WINDOW_KEEPRATIO);
    }

    while (running_) {

        for (const auto &window_component : window_components_) {
            std::string window_name = window_component.first;
            cv::Mat image;

            for (auto &priority_component : window_component.second) {
                for (auto &component : priority_component.second) {
                    component->Draw(image);
                }
            }

            if (!image.empty())
                cv::imshow(window_name, image);

        }

        cv::waitKey(10);

    }
}
traact::component::vision::OpenCVModule::OpenCVModule() {}

void traact::component::vision::OpenCVModule::addComponent(std::string window_name, OpenCVComponent *component,
                                                           size_t priority) {
    window_components_[window_name][priority].push_back(component);
}

traact::component::vision::OpenCVComponent::OpenCVComponent(const std::string &name)
    : ModuleComponent(name, ComponentType::SYNC_SINK, ModuleType::GLOBAL) {}
std::string traact::component::vision::OpenCVComponent::getModuleKey() {
    return "GlobalOpenCVModule";
}
traact::component::Module::Ptr traact::component::vision::OpenCVComponent::instantiateModule() {
    return std::make_shared<OpenCVModule>();
}
bool traact::component::vision::OpenCVComponent::configure(const nlohmann::json &parameter,
                                                           buffer::ComponentBufferConfig *data) {
    opencv_module_ = std::dynamic_pointer_cast<OpenCVModule>(module_);
    return ModuleComponent::configure(parameter, data);
}
