cmake_minimum_required(VERSION 3.16)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CONAN_TRAACT_RUN_ENV_ROOT}/cmake)

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

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
        src/traact/component/render/RenderModule.cpp
        src/traact/component/render/RenderModule.h
        src/traact/component/render/RenderImage.cpp
        src/traact/component/render/RenderPosition2DList.cpp
        src/traact/component/render/RenderPosition3DList.cpp
        src/traact/component/render/RenderPose6D.cpp
        src/traact/component/vision/OpenCVUndistort2DPoints.cpp
        src/traact/component/tracking/OutsideInPoseEstimation.cpp
        src/traact/component/spatial/Pose6DPrint.cpp
        src/traact/component/render/traact_opengl.h
        src/traact/component/vision/OpenCvBlobDetection.cpp
        src/traact/component/render/RenderKeyPointList.cpp
        src/traact/component/render/RenderModuleComponent.cpp
        src/traact/component/render/RenderModuleComponent.h
        src/traact/component/tracking/GatherCameraInitPose.cpp
        src/traact/component/vision/OpenCvArucoTracker.cpp
        src/traact/component/tracking/OutsideInPointEstimation.cpp src/traact/component/spatial/MultiplicationPose6DPosition3DList.cpp src/traact/component/tracking/FindTargetInPosition3DList.cpp)


set(IMGUI_BINDING_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/imgui_bindings/imgui_impl_glfw.h
        ${CMAKE_CURRENT_BINARY_DIR}/imgui_bindings/imgui_impl_glfw.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/imgui_bindings/imgui_impl_opengl3.h
        ${CMAKE_CURRENT_BINARY_DIR}/imgui_bindings/imgui_impl_opengl3.cpp)

find_package(imgui CONFIG)
find_package(glfw3 CONFIG)
find_package(glew CONFIG)

add_library(${CONAN_PACKAGE_NAME} SHARED
        ${IMGUI_BINDING_SOURCES}
        ${BASIC_COMPONENT_SRC}
        )

target_compile_definitions(${CONAN_PACKAGE_NAME} PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

target_include_directories(${CONAN_PACKAGE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/imgui_bindings>
        ${CONAN_INCLUDE_DIRS_NLOHMANN_JSON}
        )

include(traact_default_library_final)