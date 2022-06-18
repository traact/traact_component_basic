/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "RenderModuleComponent.h"

namespace traact::component::render {
RenderComponent::RenderComponent(const std::string &name) : ModuleComponent(name,
                                                                            ModuleType::GLOBAL) {

}

std::string RenderComponent::getModuleKey() {
    return "GlobalRenderModule";
}

Module::Ptr RenderComponent::instantiateModule() {
    return std::make_shared<RenderModule>();
}

bool RenderComponent::configure(const pattern::instance::PatternInstance &pattern_instance,
                                buffer::ComponentBufferConfig *data) {
    render_module_ = std::dynamic_pointer_cast<RenderModule>(module_);
    pattern::setValueFromParameter(pattern_instance, "Window", window_name_, "invalid");
    pattern::setValueFromParameter(pattern_instance, "Priority", priority_, 2000);
    render_module_->addComponent(window_name_, getName(), this);
    return ModuleComponent::configure(pattern_instance, data);
}

bool RenderComponent::processTimePointWithInvalid(buffer::ComponentBuffer &data) {
    auto command = std::make_shared<RenderCommand>(window_name_,
                                                   getName(),
                                                   data.getTimestamp().time_since_epoch().count(),
                                                   priority_);
    render_module_->setComponentReady(command);

    return true;
}
void RenderComponent::renderInit() {

}
void RenderComponent::renderStop() {

}

}