# /usr/bin/python3
import os
from conan import ConanFile
from conan.tools.build import can_run

class TraactPackage(ConanFile):
    python_requires = "traact_base/0.0.0@traact/latest"
    python_requires_extend = "traact_base.TraactPackageCmake"

    name = "traact_component_basic"
    version = "0.0.0"
    description = "Basic components for spatial and vision datatypes"
    url = "https://github.com/traact/traact_component_basic.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "src/*", "tests/*", "CMakeLists.txt"

    options = {
        "shared": [True, False],
        "trace_logs_in_release": [True, False]
    }

    default_options = {
        "shared": True,
        "trace_logs_in_release": True
    }

    def requirements(self):
        self.requires("traact_spatial/0.0.0@traact/latest")
        self.requires("traact_vision/0.0.0@traact/latest")        
        self.requires("apriltag/3.1.4")
        self.requires("taskflow/3.4.0")

        #self.requires("imgui/cci.20220207+1.87.docking")
        #self.requires("glfw/3.3.4")
        #self.requires("glew/2.2.0")
        
        

    #def imports(self):
    #    self.copy(src="./res/bindings", pattern="imgui_impl_glfw.*", dst="imgui_bindings", root_package='imgui')
    #    self.copy(src="./res/bindings", pattern="imgui_impl_opengl3*", dst="imgui_bindings", root_package='imgui')
    def _after_package_info(self):
        self.cpp_info.libs = ["traact_component_basic"]