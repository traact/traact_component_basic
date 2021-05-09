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

#ifndef TRAACTMULTI_RENDERMODULE_H
#define TRAACTMULTI_RENDERMODULE_H

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <map>
#include <queue>
#include <traact/util/Semaphore.h>
#include <thread>
#include <future>

namespace traact::component::render {
    class RenderComponent;

    class RenderCommand {
    public:
        typedef std::shared_ptr<RenderCommand> Ptr;
        typedef std::function<void (void)> CalledFromRenderer;
        RenderCommand(std::string window_name, std::string component_name, std::size_t mea_idx, std::size_t priority, CalledFromRenderer callback);
        RenderCommand(std::string window_name, std::string component_name, std::size_t mea_idx, std::size_t priority);
        void DoRender();
        [[nodiscard]] const std::string& GetWindowName() const;
        [[nodiscard]] const std::string& GetComponentName() const;
        [[nodiscard]] std::size_t GetMeaIdx() const;
        [[nodiscard]] std::size_t GetPriority() const;
    private:
        std::string window_name_;
        std::string component_name_;
        std::size_t mea_idx_;
        std::size_t priority_;

        CalledFromRenderer callback_;
    };

    class RenderModule : public Module {
    public:
        RenderModule();

        bool init(ComponentPtr module_component) override;

        bool start(ComponentPtr module_component) override;

        bool stop(ComponentPtr module_component) override;

        bool teardown(ComponentPtr module_component) override;

        std::size_t addComponent(std::string window_name, const std::string &component_name, RenderComponent *component,
                                 std::size_t priority);

        void setComponentReady(RenderCommand::Ptr render_command);

    private:
        std::mutex data_lock_;
        std::thread thread_;
        std::promise<void> initialized_promise_;
        bool running_{false};
        void thread_loop() ;

        std::map<std::string, std::vector<RenderComponent*> > window_components_;
        std::map<std::string, std::map<std::size_t , std::vector< RenderCommand::Ptr  > > > render_commands_;
        std::map<std::string, std::vector< RenderCommand::Ptr  > > current_render_commands_;
        std::vector<std::string> all_render_component_names_;


        WaitForInit init_lock_;

    RTTR_ENABLE(Module)
    };

    class RenderComponent : public ModuleComponent {
    public:
        explicit RenderComponent(const std::string &name);
        std::string GetModuleKey() override;
        Module::Ptr InstantiateModule() override;
        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override;
        virtual void RenderInit() = 0;
        //virtual void Draw(TimestampType ts) = 0;

        void invalidTimePoint(TimestampType ts, std::size_t mea_idx) override;

    protected:
        std::shared_ptr<RenderModule> render_module_;
        std::string window_name_;
        std::size_t priority_;
    RTTR_ENABLE(ModuleComponent)
    };


    template<class T>
    class AsyncRenderComponent : public RenderComponent {
    public:

        explicit AsyncRenderComponent(const std::string &name) : RenderComponent(name) {

        };

        void Draw(TimestampType ts)  {
            T* tmp(nullptr);

            {
                std::scoped_lock lock(data_lock_);
                auto result = data_.find(ts);
                if(result != data_.end()){
                    tmp = result->second;
                    data_.erase(result);
                }
            }

            if(tmp) {
                DrawNow(ts, *tmp);
                releaseAsyncCall(ts);
            }



        }

        virtual void DrawNow(TimestampType ts, const T& data) = 0;

    protected:
        std::mutex data_lock_;

        std::map<TimestampType, const T*> data_;
    RTTR_ENABLE(RenderComponent)
    };

}




#endif //TRAACTMULTI_RENDERMODULE_H
