cmake_minimum_required(VERSION 3.16)

set(TARGET_NAME traact_component_basic)

project(${TARGET_NAME} VERSION "0.0.0" LANGUAGES CXX)

find_package(traact_base)
list(APPEND CMAKE_MODULE_PATH ${traact_base_INCLUDE_DIR})

include(traact_default_library_setup)

set(BASIC_COMPONENT_SRC
        src/traact/component/spatial/util/Pose6DTestSource.h
        src/traact/component/spatial/Multiplication.cpp
        src/traact/component/spatial/StaticPose.cpp
        src/traact/component/spatial/InversionComponent.cpp
        src/traact/component/vision/OpenCvColorToGray.cpp
        src/traact/component/vision/OpenCvVideoCapture.cpp
        src/traact/component/vision/OpenCvUndistortImage.cpp
        src/traact/component/vision/OpenCvConvertImage.cpp
        src/traact/component/tracking/RefineCircles.cpp
        src/traact/component/tracking/BruteForceEstimatePose.cpp
        #src/traact/component/render/RenderModule.cpp
        #src/traact/component/render/RenderModule.h
        #src/traact/component/render/RenderImage.cpp
        #src/traact/component/render/RenderPosition2DList.cpp
        #src/traact/component/render/RenderPosition3DList.cpp
        #src/traact/component/render/RenderPose6D.cpp
        #src/traact/component/render/traact_opengl.h
        #src/traact/component/render/RenderKeyPointList.cpp
        #src/traact/component/render/RenderModuleComponent.cpp
        #src/traact/component/render/RenderModuleComponent.h
        src/traact/component/vision/OpenCVUndistort2DPoints.cpp
        src/traact/component/tracking/OutsideInPoseEstimation.cpp
        src/traact/component/spatial/Pose6DPrint.cpp        
        src/traact/component/vision/OpenCvBlobDetection.cpp        
        src/traact/component/tracking/GatherCameraInitPose.cpp
        src/traact/component/vision/OpenCvArucoTracker.cpp
        src/traact/component/tracking/OutsideInPointEstimation.cpp 
        src/traact/component/spatial/MultiplicationPose6DPosition3DList.cpp 
        src/traact/component/tracking/FindTargetInPosition3DList.cpp
        src/traact/component/vision/OpenCvCudaEncode.cpp
        src/traact/component/vision/OpenCvCudaDownload.cpp 
        src/traact/component/vision/ApriltagTracker.cpp
        src/traact/component/vision/OpenCvCudaDecode.cpp 
        src/traact/component/vision/OpenCvCudaUpload.cpp 
        src/traact/component/vision/OpenCvCudaConvertImage.cpp 
        src/traact/component/vision/OpenCvCudaUndistortImage.cpp
        src/traact/component/vision/VisionGpuFunctions.cu 
        src/traact/component/vision/VisionGpuFunctions.cuh 
        src/traact/component/vision/OpenCvCudaCvtColor.cpp
        )

find_package(traact_spatial REQUIRED)
find_package(traact_vision REQUIRED)
find_package(apriltag REQUIRED)
find_package(Taskflow)


add_library(${TARGET_NAME} SHARED ${BASIC_COMPONENT_SRC})
target_link_libraries(${TARGET_NAME} traact_spatial::traact_spatial traact_vision::traact_vision opencv::opencv apriltag::apriltag Taskflow::Taskflow) 


include(traact_default_library_final)