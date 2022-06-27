/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_GUI_SRC_TRAACT_GUI_DEBUG_RUN_TRAACT_OPENGL_H_
#define TRAACT_GUI_SRC_TRAACT_GUI_DEBUG_RUN_TRAACT_OPENGL_H_

#include <traact/vision.h>
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>

namespace traact {

static const constexpr GLint getOpenGl(vision::PixelFormat pixel_format) {
    switch (pixel_format) {
        case vision::PixelFormat::LUMINANCE:return GL_LUMINANCE;
        case vision::PixelFormat::RGB: return GL_RGB;
        case vision::PixelFormat::BGR: return GL_BGR;
        case vision::PixelFormat::RGBA: return GL_RGBA;
        case vision::PixelFormat::BGRA:return GL_BGRA;
        case vision::PixelFormat::DEPTH:return GL_R;
        case vision::PixelFormat::FLOAT:return GL_R;

        case vision::PixelFormat::YUV422:
        case vision::PixelFormat::YUV411:
        case vision::PixelFormat::RAW:
        case vision::PixelFormat::MJPEG:
        case vision::PixelFormat::UNKNOWN_PIXELFORMAT:return GL_INVALID_ENUM;
    }
}

static const constexpr GLint getOpenGl(BaseType data_type) {
    switch (data_type) {

        case INT_8: return GL_BYTE;
        case UINT_8:return GL_UNSIGNED_BYTE;
        case INT_16:return GL_SHORT;
        case UINT_16:return GL_UNSIGNED_SHORT;
        case INT_32:return GL_INT;
        case FLOAT_16:return GL_HALF_FLOAT;
        case FLOAT_32:return GL_FLOAT;
        case FLOAT_64:return GL_DOUBLE;
        case UNKNOWN: return GL_INVALID_ENUM;

    }
}

static void uploadImageOpenGL(const GLuint texture,
                        const GLvoid *data,
                        GLint internalFormat,
                        const vision::ImageHeader &header) {
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);


    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, header.width, header.height, 0,
                 getOpenGl(header.pixel_format), getOpenGl(header.base_type), data);
}
}

#endif //TRAACT_GUI_SRC_TRAACT_GUI_DEBUG_RUN_TRAACT_OPENGL_H_
