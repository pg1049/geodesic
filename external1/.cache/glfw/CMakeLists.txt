# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(glfw-download NONE)

include(ExternalProject)
ExternalProject_Add(glfw-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/glfw/glfw.git;GIT_TAG;3327050ca66ad34426a82c217c2d60ced61526b7
                    SOURCE_DIR          "G:/geodesic/cmake/../external/glfw"
                    BINARY_DIR          "G:/geodesic/build/glfw-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
