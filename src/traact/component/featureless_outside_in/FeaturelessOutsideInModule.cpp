/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#include "FeaturelessOutsideInModule.h"
#include <traact/buffer/SourceComponentBuffer.h>
traact::component::FeaturelessOutsideInModule::FeaturelessOutsideInModule() {

}

bool traact::component::FeaturelessOutsideInModule::init(traact::component::Module::ComponentPtr module_component) {
    return Module::init(module_component);
}

bool traact::component::FeaturelessOutsideInModule::start(traact::component::Module::ComponentPtr module_component) {
    return Module::start(module_component);
}

bool traact::component::FeaturelessOutsideInModule::stop(traact::component::Module::ComponentPtr module_component) {
    return Module::stop(module_component);
}

bool traact::component::FeaturelessOutsideInModule::teardown(traact::component::Module::ComponentPtr module_component) {
    return Module::teardown(module_component);
}

void traact::component::FeaturelessOutsideInModule::addComponent(
    traact::component::FeaturelessOutsideInComponent *component) {

}

void traact::component::FeaturelessOutsideInModule::setCameraReady(
    traact::component::FeaturelessOutsideInComponent *component, traact::spatial::Pose6D *camera2world,
    traact::vision::CameraCalibration *calibration, traact::spatial::Position2DList *data) {

}

traact::component::FeaturelessOutsideInComponent::FeaturelessOutsideInComponent(const std::string &name,
                                                                                traact::component::ComponentType traact_component_type)
    : ModuleComponent(name, traact_component_type, ModuleType::GLOBAL) {

}

std::string traact::component::FeaturelessOutsideInComponent::getModuleKey() {
    return "FeaturelessOutsideInModule";
}

traact::component::Module::Ptr traact::component::FeaturelessOutsideInComponent::instantiateModule() {
    return std::make_shared<FeaturelessOutsideInModule>();
}

bool traact::component::FeaturelessOutsideInComponent::configure(const nlohmann::json &parameter,
                                                                 traact::buffer::ComponentBufferConfig *data) {
    tracking_module_ = std::dynamic_pointer_cast<FeaturelessOutsideInModule>(module_);
    tracking_module_->addComponent(this);
    return ModuleComponent::configure(parameter, data);
}

traact::component::FeaturelessOutsideInComponentInput::FeaturelessOutsideInComponentInput(const std::string &name)
    : FeaturelessOutsideInComponent(name, ComponentType::ASYNC_FUNCTIONAL) {

}

void
traact::component::FeaturelessOutsideInComponentInput::invalidTimePoint(traact::Timestamp ts, size_t mea_idx) {
    this->releaseAsyncCall(ts, false);
}

traact::component::FeaturelessOutsideInComponentOutput::FeaturelessOutsideInComponentOutput(const std::string &name)
    : FeaturelessOutsideInComponent(name, ComponentType::ASYNC_SOURCE) {

}

void traact::component::FeaturelessOutsideInComponentOutput::invalidTimePoint(traact::Timestamp ts,
                                                                              size_t mea_idx) {
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if (!buffer) {
        SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
        return;
    }
    buffer->commit(false);

}
