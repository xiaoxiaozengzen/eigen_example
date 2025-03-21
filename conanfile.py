import os
import shutil
import subprocess
import re
from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.env import VirtualRunEnv, VirtualBuildEnv
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.scm import Version

required_conan_version = ">=1.53.0"

class EigenTestConan(ConanFile):
    name = "eigen_test"
    version = "dev"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "Debug": [True, False],
    }
    default_options = {
        "Debug": True,
    }


    @property
    def _min_cppstd(self):
        return 14

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("nlohmann_json/3.11.2")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["CMAKE_POLICY_DEFAULT_CMP0077"] = "NEW"
        if self.options.Debug:
            tc.variables["BUILD_DEBUG"] = True
        else:
            tc.variables["BUILD_DEBUG"] = False
        tc.generate()
        tc = CMakeDeps(self)
        tc.generate()
        tc = VirtualRunEnv(self)
        tc.generate()
        tc = VirtualBuildEnv(self)
        tc.generate(scope="build")
