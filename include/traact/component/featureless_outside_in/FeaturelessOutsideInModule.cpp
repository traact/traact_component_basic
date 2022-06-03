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
                                                                                traact::component::ComponentType traact_component_type) : ModuleComponent(name, traact_component_type, ModuleType::Global) {

}

std::string traact::component::FeaturelessOutsideInComponent::GetModuleKey() {
    return "FeaturelessOutsideInModule";
}

traact::component::Module::Ptr traact::component::FeaturelessOutsideInComponent::InstantiateModule() {
    return std::make_shared<FeaturelessOutsideInModule>();
}

bool traact::component::FeaturelessOutsideInComponent::configure(const nlohmann::json &parameter,
                                                                 traact::buffer::ComponentBufferConfig *data) {
    tracking_module_ = std::dynamic_pointer_cast<FeaturelessOutsideInModule>(module_);
    tracking_module_->addComponent(this);
    return ModuleComponent::configure(parameter, data);
}


traact::component::FeaturelessOutsideInComponentInput::FeaturelessOutsideInComponentInput(const std::string &name) : FeaturelessOutsideInComponent(name, ComponentType::AsyncFunctional) {

}

void
traact::component::FeaturelessOutsideInComponentInput::invalidTimePoint(traact::TimestampType ts, std::size_t mea_idx) {
    this->releaseAsyncCall(ts, false);
}

traact::component::FeaturelessOutsideInComponentOutput::FeaturelessOutsideInComponentOutput(const std::string &name) : FeaturelessOutsideInComponent(name, ComponentType::AsyncSource) {

}

void traact::component::FeaturelessOutsideInComponentOutput::invalidTimePoint(traact::TimestampType ts,
                                                                              std::size_t mea_idx) {
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if(!buffer){
        SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
        return;
    }
    buffer->Commit(false);

}
