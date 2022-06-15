/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

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
    : ModuleComponent(name, ModuleType::GLOBAL) {

}

std::string traact::component::FeaturelessOutsideInComponent::getModuleKey() {
    return "FeaturelessOutsideInModule";
}

traact::component::Module::Ptr traact::component::FeaturelessOutsideInComponent::instantiateModule() {
    return std::make_shared<FeaturelessOutsideInModule>();
}

bool traact::component::FeaturelessOutsideInComponent::configure(const pattern::instance::PatternInstance &pattern_instance,
                                                                 traact::buffer::ComponentBufferConfig *data) {
    tracking_module_ = std::dynamic_pointer_cast<FeaturelessOutsideInModule>(module_);
    tracking_module_->addComponent(this);
    return ModuleComponent::configure(pattern_instance, data);
}

traact::component::FeaturelessOutsideInComponentInput::FeaturelessOutsideInComponentInput(const std::string &name)
    : FeaturelessOutsideInComponent(name, ComponentType::ASYNC_FUNCTIONAL) {

}

bool
traact::component::FeaturelessOutsideInComponentInput::processTimePointWithInvalid(buffer::ComponentBuffer &data) {
    this->releaseAsyncCall(data.getTimestamp(), false);
}

traact::component::FeaturelessOutsideInComponentOutput::FeaturelessOutsideInComponentOutput(const std::string &name)
    : FeaturelessOutsideInComponent(name, ComponentType::ASYNC_SOURCE) {

}

bool traact::component::FeaturelessOutsideInComponentOutput::processTimePointWithInvalid(buffer::ComponentBuffer &data) {
    auto buffer_future = request_callback_(data.getTimestamp());
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if (!buffer) {
        SPDLOG_WARN("Could not get source buffer for ts {0}", data.getTimestamp().time_since_epoch().count());
        return false;
    }
    buffer->commit(false);
    return true;
}
