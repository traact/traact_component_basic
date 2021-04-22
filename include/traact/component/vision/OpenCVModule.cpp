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

#include "OpenCVModule.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
bool traact::component::vision::OpenCVModule::init(traact::component::Module::ComponentPtr module_component) {
  return Module::init(module_component);
}
bool traact::component::vision::OpenCVModule::start(traact::component::Module::ComponentPtr module_component) {
//    tbb::queuing_mutex::scoped_lock lock(running_lock_);
  if(running_)
    return true;
  SPDLOG_DEBUG("Starting OpenCV Module");
  running_ = true;
  thread_ = std::make_shared<std::thread>([this]{
    thread_loop();
  });
  //init_lock_.Wait();

  return Module::start(module_component);
}
bool traact::component::vision::OpenCVModule::stop(traact::component::Module::ComponentPtr module_component) {
   // tbb::queuing_mutex::scoped_lock lock(running_lock_);
  if(!running_)
    return true;
  running_ = false;
  thread_->join();
  return Module::stop(module_component);
}
bool traact::component::vision::OpenCVModule::teardown(traact::component::Module::ComponentPtr module_component) {
  return Module::teardown(module_component);
}

void traact::component::vision::OpenCVModule::thread_loop() {

    for(const auto& window_component : window_components_) {
        cv::namedWindow(window_component.first, cv::WINDOW_KEEPRATIO);
    }


  while (running_) {

      for(const auto& window_component : window_components_) {
          std::string window_name = window_component.first;
          cv::Mat image;

          for(auto& priority_component : window_component.second){
              for(auto& component : priority_component.second){
                  component->Draw(image);
              }
          }

          if(!image.empty())
              cv::imshow(window_name, image);



      }



    cv::waitKey(10);


  }
}
traact::component::vision::OpenCVModule::OpenCVModule() {}

void traact::component::vision::OpenCVModule::addComponent(std::string window_name, OpenCVComponent *component,
                                                           std::size_t priority) {
   window_components_[window_name][priority].push_back(component);
}



traact::component::vision::OpenCVComponent::OpenCVComponent(const std::string &name)
    : ModuleComponent(name,ComponentType::SyncSink, ModuleType::Global) {}
std::string traact::component::vision::OpenCVComponent::GetModuleKey() {
  return "GlobalOpenCVModule";
}
traact::component::Module::Ptr traact::component::vision::OpenCVComponent::InstantiateModule() {
  return std::make_shared<OpenCVModule>();
}
bool traact::component::vision::OpenCVComponent::configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) {
  opencv_module_ = std::dynamic_pointer_cast<OpenCVModule>(module_);
  return ModuleComponent::configure(parameter, data);
}
