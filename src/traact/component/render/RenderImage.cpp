/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "RenderModuleComponent.h"

#include <opencv2/imgproc.hpp>
#include "traact_opengl.h"

namespace traact::component::render {

//class RenderImage : public AsyncRenderComponent<traact::vision::ImageHeader::NativeType> {
class RenderImage : public RenderComponent {
 public:
    using InPortImage = buffer::PortConfig<vision::ImageHeader, 0>;
    RenderImage(const std::string &name)
        : RenderComponent(name) {}

    static traact::pattern::Pattern::Ptr GetPattern() {
        using namespace traact::vision;
        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("RenderImage", Concurrency::SERIAL, ComponentType::SYNC_SINK);

        pattern->addStringParameter("Window", "RenderWindow")
        .addParameter("Priority", 1000);

        pattern->addConsumerPort<InPortImage>("input");
        pattern->addCoordinateSystem("A").addCoordinateSystem("B").addEdge("Camera", "ImagePlane", "input");

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance,
                   buffer::ComponentBufferConfig *data) override {
        render_module_ = std::dynamic_pointer_cast<RenderModule>(module_);
        pattern::setValueFromParameter(pattern_instance, "Window", window_name_, getName());
        pattern::setValueFromParameter(pattern_instance, "Priority", priority_, 1000);
        render_module_->addComponent(window_name_, getName(), this);
        return module_->init(this);
    }

    bool start() override {
        last_ts_ = nowSteady();
        fps_ = 0;
        return ModuleComponent::start();
    }

    bool processTimePoint(traact::buffer::ComponentBuffer &data) override {
        using namespace traact::vision;


        auto now_steady = nowSteady();
        auto diff = now_steady - last_ts_;
        last_ts_ = now_steady;
        auto diff_double = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1> > >(diff);
        fps_ = fps_ * 0.9 + 0.1 * 1.0 / diff_double.count();
        ++valid_count_;

        auto image_index = data.getTimestamp().time_since_epoch().count();

        render_module_->addAdditionalCommand(std::make_shared<RenderCommand>(
            window_name_, getName(),
            image_index, priority_ / 2,
            [this, &data]()  {
                uploadImage(data);
            }));

        latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                          image_index, priority_,
                                                          [this, image_index]() { draw(image_index); });

        render_module_->setComponentReady(latest_command_);

//        if(mapped_image_buffer_ != nullptr){
//            const auto image = data.getInput<InPortImage>().getImage();
//            auto image_header = data.getInputHeader<InPortImage>();
//            auto data_size = image_header.width * image_header.height * image_header.channels;// * getBytes(image_header.base_type);
//            memcpy(mapped_image_buffer_, image.data, data_size);
//        }

        return true;

    }

    virtual bool processTimePointWithInvalid(buffer::ComponentBuffer &data) override {
        if(latest_command_){
            latest_command_->updateMeaIdxForReuse(data.getTimestamp().time_since_epoch().count());
        } else {
            latest_command_ = std::make_shared<RenderCommand>(window_name_, getName(),
                                                              data.getTimestamp().time_since_epoch().count(), priority_,
                                                              []() {  });
        }

        render_module_->setComponentReady(latest_command_);
        return true;
    }

    virtual void renderStop() override {
        if(init_texture_){
            glDeleteTextures(1, &texture_);
            glDeleteBuffers(1, &pbo_id_);
        }
    }

    void uploadImage(traact::buffer::ComponentBuffer &data)  {
        const auto image = data.getInput<InPortImage>().getImage();
        auto image_header = data.getInputHeader<InPortImage>();
        SPDLOG_TRACE("{0}: uploadImage {1}", getName(), data.getTimestamp());

        if(data.getTimestamp() < last_image_upload_){
            SPDLOG_ERROR("image upload of older image last: {0} current: {1}", last_image_upload_, data.getTimestamp());
        }
        last_image_upload_ = data.getTimestamp();
        ++upload_count_;

//        if(mapped_image_buffer_ != nullptr){
//            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, (index_ + 1) % 2);
//            glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
//        }

        index_ = (index_ + 1) % 2;
        auto nextIndex = (index_ + 1) % 2;

        auto data_size = image_header.width * image_header.height * image_header.channels * getBytes(image_header.base_type);
        if (!init_texture_) {
            glGenTextures(1, &texture_);
            glBindTexture(GL_TEXTURE_2D, texture_);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glBindTexture(GL_TEXTURE_2D, 0);

            glGenBuffers(1, &pbo_id_);
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_id_);
            glBufferData(GL_PIXEL_UNPACK_BUFFER, data_size, 0, GL_STREAM_DRAW);
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

            render_module_->setImageSize(ImVec2(image_header.width, image_header.height), window_name_);

            init_texture_ = true;
        }
        glBindTexture(GL_TEXTURE_2D, texture_);
        //glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_id_);
        glBufferData(GL_PIXEL_UNPACK_BUFFER, data_size, image.data, GL_STREAM_DRAW);
//        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_header.width, image_header.height,
//                        getOpenGl(image_header.pixel_format), GL_UNSIGNED_BYTE, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_header.width, image_header.height, 0,
                     getOpenGl(image_header.pixel_format), getOpenGl(image_header.base_type), 0);
        glBindTexture(GL_TEXTURE_2D, 0);

        //glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo_id_[nextIndex]);
        //glBufferData(GL_PIXEL_UNPACK_BUFFER, data_size, 0, GL_STREAM_DRAW);
        //glBufferData(GL_PIXEL_UNPACK_BUFFER, data_size, image.data, GL_STREAM_DRAW);
        //mapped_image_buffer_ = (GLubyte*)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
//        if(mapped_image_buffer_)
//        {
//            memcpy(mapped_image_buffer_, image.data, data_size);
//            glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);  // release pointer to mapping buffer
//        }


        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
        //uploadImageOpenGL(texture_, image.data, GL_RGB, image_header);

    }

    void draw(long image_index) {
        ImVec2 avail_size = ImGui::GetContentRegionAvail();
        if (avail_size.x < 10) {
            avail_size.x = 320;
            avail_size.y = 180;
        }

        render_module_->setImageRenderSize(avail_size, window_name_);

        glBindTexture(GL_TEXTURE_2D, texture_);
        ImGui::Image(reinterpret_cast<void *>( static_cast<intptr_t>( texture_ )), avail_size);

//        ImGui::Begin("Stats");
//        ImGui::Text("%s index: %ld  fps: %f diff: %d",
//                    window_name_.c_str(),
//                    image_index,
//                    fps_,
//                    upload_count_ - valid_count_);
//        ImGui::End();
    }

 private:
    GLuint texture_;
    bool init_texture_{false};
    double fps_{0};
    TimestampSteady last_ts_{TimestampSteady::min()};
    int valid_count_{0};
    int upload_count_{0};
    Timestamp last_image_upload_{kTimestampZero};
    GLuint pbo_id_;
    int index_;
    GLubyte* mapped_image_buffer_{nullptr};

};

CREATE_TRAACT_COMPONENT_FACTORY(RenderImage)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::render::RenderImage)
END_TRAACT_PLUGIN_REGISTRATION
