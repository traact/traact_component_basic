# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "traact_component_basic"
    description = "Basic components for spatial and vision datatypes"
    url = "https://github.com/traact/traact_component_basic.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    keep_imports = True
    exports_sources = "src/*", "tests/*", "CMakeLists.txt"

    def requirements(self):
        self.traact_requires("traact_spatial", "latest")
        self.traact_requires("traact_vision", "latest")
        self.requires("spdlog/1.10.0")
        #self.requires("imgui/1.83")
        self.requires("imgui/cci.20220207+1.87.docking")
        self.requires("glfw/3.3.4")
        self.requires("glew/2.2.0")
        if self.options.with_tests:
            self.requires("gtest/[>=1.11.0]")

    def imports(self):
        self.copy(src="./res/bindings", pattern="imgui_impl_glfw.*", dst="imgui_bindings", root_package='imgui')
        self.copy(src="./res/bindings", pattern="imgui_impl_opengl3*", dst="imgui_bindings", root_package='imgui')
