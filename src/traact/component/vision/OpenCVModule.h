/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVMODULE_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVMODULE_H_

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/highgui.hpp>
#include <map>
#include <queue>
#include <traact/util/Semaphore.h>
#include <thread>

namespace traact::component::vision {

class OpenCVComponent;

class OpenCVModule : public Module {
 public:
    OpenCVModule();
    bool init(ComponentPtr module_component) override;
    bool start(ComponentPtr module_component) override;
    bool stop(ComponentPtr module_component) override;
    bool teardown(ComponentPtr module_component) override;

    void addComponent(std::string window_name, OpenCVComponent *component, size_t priority);


    //void updateWindow(const std::string& window_name, const cv::Mat& image);

 private:
    std::shared_ptr<std::thread> thread_;
    bool running_{false};
    void thread_loop();
    std::map<std::string, std::map<size_t, std::vector<OpenCVComponent *> > > window_components_;

    WaitForInit init_lock_;

 RTTR_ENABLE(Module)
};

class OpenCVComponent : public ModuleComponent {
 public:
    OpenCVComponent(const std::string &name);
    std::string getModuleKey() override;
    Module::Ptr instantiateModule() override;
    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override;
    virtual void Draw(cv::Mat &image) = 0;
 protected:
    std::shared_ptr<OpenCVModule> opencv_module_;
 RTTR_ENABLE(ModuleComponent)
};

}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVMODULE_H_
