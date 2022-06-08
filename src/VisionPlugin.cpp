/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <rttr/registration>
#include <rttr/type>

#include "traact/vision.h"
#include <traact/component/facade/ApplicationAsyncSource.h>
#include <traact/component/facade/ApplicationSyncSink.h>

namespace traact::vision {

}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::vision::CameraCalibrationFactoryObject>("CameraCalibrationFactoryObject").constructor<>()();
    registration::class_<traact::vision::ImageFactoryObject>("ImageFactoryObject").constructor<>()();

    registration::class_<traact::component::facade::ApplicationAsyncSource<traact::vision::ImageHeader> >(
        "ApplicationAsyncSource_ImageHeader").constructor<std::string>()();
    registration::class_<traact::component::facade::ApplicationSyncSink<traact::vision::CameraCalibrationHeader> >(
        "ApplicationSyncSink_CameraCalibrationHeader").constructor<std::string>()();
}