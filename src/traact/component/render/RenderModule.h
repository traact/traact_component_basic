/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_COMPONENT_BASIC_SRC_TRAACT_COMPONENT_RENDER_RENDERMODULE_H_
#define TRAACT_COMPONENT_BASIC_SRC_TRAACT_COMPONENT_RENDER_RENDERMODULE_H_
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_internal.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <future>
#include <map>
#include <queue>
#include <thread>
#include <traact/spatial.h>
#include <traact/traact.h>
#include <traact/util/Semaphore.h>
#include <traact/vision.h>
#include <traact/util/FpsCounter.h>



namespace traact::component::render {

struct RenderConfiguration {
    float target_fps;
};

class RenderComponent;

class RenderCommand {
 public:
    typedef std::shared_ptr<RenderCommand> Ptr;
    typedef std::function<void(void)> CalledFromRenderer;
    RenderCommand(std::string window_name,
                  std::string component_name,
                  size_t mea_idx,
                  size_t priority,
                  CalledFromRenderer callback);
    RenderCommand(std::string window_name, std::string component_name, size_t mea_idx, size_t priority);
    void DoRender();
    [[nodiscard]] const std::string &GetWindowName() const;
    [[nodiscard]] const std::string &GetComponentName() const;
    [[nodiscard]] size_t GetMeaIdx() const;
    [[nodiscard]] size_t GetPriority() const;
    void updateMeaIdxForReuse(size_t new_mea_idx);
 private:
    std::string window_name_;
    std::string component_name_;
    size_t mea_idx_;
    size_t priority_;

    CalledFromRenderer callback_;
};

class RenderModule : public Module {
 public:
    RenderModule();

    bool init(ComponentPtr module_component) override;

    bool start(ComponentPtr module_component) override;

    bool stop(ComponentPtr module_component) override;

    bool teardown(ComponentPtr module_component) override;

    void processTimePoint() override;

    size_t addComponent(std::string window_name,
                        const std::string &component_name,
                        RenderComponent *component,
                        RenderConfiguration render_config = {60.0f});

    void addAdditionalCommand(RenderCommand::Ptr render_command);
    void setComponentReady(RenderCommand::Ptr render_command);

    void setImageRenderSize(ImVec2 render_size, const std::string &window);
    void setImageSize(ImVec2 image_size, const std::string &window);
    void setCameraCalibration(const vision::CameraCalibration &camera_calibration, const std::string &window);
    const std::optional<ImVec2> &getImageRenderSize(const std::string &window) const;
    const std::optional<ImVec2> &getImageSize(const std::string &window) const;
    const std::optional<vision::CameraCalibration>& getCameraCalibration(const std::string &window) const;


 private:
    std::mutex data_lock_;
    std::thread thread_;
    std::promise<void> initialized_promise_;
    std::atomic_bool running_{false};
    WaitForInit additional_commands_processed_;

    std::map<std::string, std::optional<ImVec2>> render_size_{};
    std::map<std::string, std::optional<ImVec2>> image_size_{};
    std::map<std::string,std::optional<vision::CameraCalibration>> camera_calibration_{};

    void threadLoop();

    std::map<std::string, std::vector<RenderComponent *> > window_components_;
    std::map<std::string, std::vector<RenderCommand::Ptr> > additional_commands_;
    std::map<std::string, std::vector<RenderCommand::Ptr> > render_commands_;
    std::map<std::string, std::vector<RenderCommand::Ptr> > current_render_commands_;
    std::map<std::string, std::vector<RenderCommand::Ptr> > current_additional_commands_;
    std::vector<std::string> all_render_component_names_;
    util::FpsCounter fps_render_;
    std::map<std::string, util::FpsCounter> fps_new_data_;
    //TimestampSteady last_render_timestamp_;
    TimeDuration target_loop_time_;
    float render_target_fps_{60};


    WaitForInit init_lock_;

    void updateCurrentRenderCommands();
};




}

#endif //TRAACT_COMPONENT_BASIC_SRC_TRAACT_COMPONENT_RENDER_RENDERMODULE_H_
