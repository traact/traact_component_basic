/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

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

    void setCameraReady(FeaturelessOutsideInComponent *component,
                        spatial::Pose6D *camera2world,
                        vision::CameraCalibration *calibration,
                        spatial::Position2DList *data);

 private:
    std::mutex data_lock_;

 RTTR_ENABLE(Module)
};

class FeaturelessOutsideInComponent : public ModuleComponent {
 public:
    explicit FeaturelessOutsideInComponent(const std::string &name, ComponentType traact_component_type);
    virtual ~FeaturelessOutsideInComponent() = default;
    std::string getModuleKey() override;
    Module::Ptr instantiateModule() override;
    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override;

 protected:
    std::shared_ptr<FeaturelessOutsideInModule> tracking_module_;
 RTTR_ENABLE(ModuleComponent)
};

class FeaturelessOutsideInComponentInput : public FeaturelessOutsideInComponent {
 public:
    explicit FeaturelessOutsideInComponentInput(const std::string &name);

    virtual void sendResult(spatial::Position2DList data) = 0;
    void invalidTimePoint(Timestamp ts, size_t mea_idx) override;

 protected:
 RTTR_ENABLE(FeaturelessOutsideInComponent)
};

class FeaturelessOutsideInComponentOutput : public FeaturelessOutsideInComponent {
 public:
    explicit FeaturelessOutsideInComponentOutput(const std::string &name);

    virtual void sendPose(spatial::Pose6D pose) = 0;
    void invalidTimePoint(Timestamp ts, size_t mea_idx) override;

 protected:
 RTTR_ENABLE(FeaturelessOutsideInComponent)
};

}

#endif //TRAACTMULTI_FEATURELESSOUTSIDEINMODULE_H
