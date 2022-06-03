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

#ifndef TRAACTMULTI_FEATURELESSOUTSIDEINMODULE_H
#define TRAACTMULTI_FEATURELESSOUTSIDEINMODULE_H


#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <map>
#include <queue>
#include <traact/util/Semaphore.h>
#include <thread>
#include <future>

namespace traact::component {
    class FeaturelessOutsideInComponent;


    class FeaturelessOutsideInModule : public Module {
    public:
        FeaturelessOutsideInModule();

        bool init(ComponentPtr module_component) override;

        bool start(ComponentPtr module_component) override;

        bool stop(ComponentPtr module_component) override;

        bool teardown(ComponentPtr module_component) override;

        void addComponent(FeaturelessOutsideInComponent *component);


        void setCameraReady(FeaturelessOutsideInComponent *component, spatial::Pose6D* camera2world, vision::CameraCalibration* calibration, spatial::Position2DList* data);

    private:
        std::mutex data_lock_;


        RTTR_ENABLE(Module)
    };

    class FeaturelessOutsideInComponent : public ModuleComponent {
    public:
        explicit FeaturelessOutsideInComponent(const std::string &name, ComponentType traact_component_type);
        virtual ~FeaturelessOutsideInComponent() = default;
        std::string GetModuleKey() override;
        Module::Ptr InstantiateModule() override;
        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override;



    protected:
        std::shared_ptr<FeaturelessOutsideInModule> tracking_module_;
        RTTR_ENABLE(ModuleComponent)
    };

    class FeaturelessOutsideInComponentInput : public FeaturelessOutsideInComponent {
    public:
        explicit FeaturelessOutsideInComponentInput(const std::string &name);

        virtual void sendResult(spatial::Position2DList data) = 0;
        void invalidTimePoint(TimestampType ts, std::size_t mea_idx) override;

    protected:
    RTTR_ENABLE(FeaturelessOutsideInComponent)
    };

    class FeaturelessOutsideInComponentOutput : public FeaturelessOutsideInComponent {
    public:
        explicit FeaturelessOutsideInComponentOutput(const std::string &name);

        virtual void sendPose(spatial::Pose6D pose) = 0;
        void invalidTimePoint(TimestampType ts, std::size_t mea_idx) override;

    protected:
        RTTR_ENABLE(FeaturelessOutsideInComponent)
    };



}


#endif //TRAACTMULTI_FEATURELESSOUTSIDEINMODULE_H
