/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_RENDERMODULE_H
#define TRAACTMULTI_RENDERMODULE_H

#include <future>
#include <imgui.h>
#include <map>
#include <queue>
#include <thread>
#include <traact/spatial.h>
#include <traact/traact.h>
#include <traact/util/Semaphore.h>
#include <traact/vision.h>

namespace traact::component::render {
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

    size_t addComponent(std::string window_name, const std::string &component_name, RenderComponent *component,
                        size_t priority);

    void setComponentReady(RenderCommand::Ptr render_command);

    void setImageRenderSize(ImVec2 render_size);
    std::optional<ImVec2> getImageRenderSize();

 private:
    std::mutex data_lock_;
    std::thread thread_;
    std::promise<void> initialized_promise_;
    bool running_{false};
    std::optional<ImVec2> render_size_{};
    void thread_loop();

    std::map<std::string, std::vector<RenderComponent *> > window_components_;
    std::map<std::string, std::map<size_t, std::vector<RenderCommand::Ptr> > > render_commands_;
    std::map<std::string, std::vector<RenderCommand::Ptr> > current_render_commands_;
    std::vector<std::string> all_render_component_names_;

    WaitForInit init_lock_;

 RTTR_ENABLE(Module)
};

class RenderComponent : public ModuleComponent {
 public:
    explicit RenderComponent(const std::string &name);
    std::string getModuleKey() override;
    Module::Ptr instantiateModule() override;
    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override;
    virtual void RenderInit() = 0;
    //virtual void Draw(Timestamp ts) = 0;

    void invalidTimePoint(Timestamp ts, size_t mea_idx) override;

 protected:
    std::shared_ptr<RenderModule> render_module_;
    std::string window_name_;
    size_t priority_;
 RTTR_ENABLE(ModuleComponent)
};

template<class T>
class AsyncRenderComponent : public RenderComponent {
 public:

    explicit AsyncRenderComponent(const std::string &name) : RenderComponent(name) {

    };

    void Draw(Timestamp ts) {
        T *tmp(nullptr);

        {
            std::scoped_lock lock(data_lock_);
            auto result = data_.find(ts);
            if (result != data_.end()) {
                tmp = result->second;
                data_.erase(result);
            }
        }

        if (tmp) {
            DrawNow(ts, *tmp);
            releaseAsyncCall(ts, false);
        }

    }

    virtual void DrawNow(Timestamp ts, const T &data) = 0;

 protected:
    std::mutex data_lock_;

    std::map<Timestamp, const T *> data_;
 RTTR_ENABLE(RenderComponent)
};

}

#endif //TRAACTMULTI_RENDERMODULE_H
