/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_COMPONENT_BASIC_SRC_TRAACT_COMPONENT_RENDER_RENDERMODULECOMPONENT_H_
#define TRAACT_COMPONENT_BASIC_SRC_TRAACT_COMPONENT_RENDER_RENDERMODULECOMPONENT_H_

#include "RenderModule.h"

namespace traact::component::render {
class RenderComponent : public ModuleComponent {
 public:
    explicit RenderComponent(const std::string &name);
    std::string getModuleKey() override;
    Module::Ptr instantiateModule() override;
    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override;
    virtual void renderInit();
    virtual void renderStop();

    bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override;

 protected:
    std::shared_ptr<RenderModule> render_module_;
    std::shared_ptr<RenderCommand> latest_command_;
    std::string window_name_;
    size_t priority_;

};
}

#endif //TRAACT_COMPONENT_BASIC_SRC_TRAACT_COMPONENT_RENDER_RENDERMODULECOMPONENT_H_
