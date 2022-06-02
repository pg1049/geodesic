# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(eigen-download NONE)

include(ExternalProject)
ExternalProject_Add(eigen-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://gitlab.com/libeigen/eigen.git;GIT_TAG;3.3.7
                    SOURCE_DIR          "G:/geodesic-master/cmake/../external/eigen"
                    BINARY_DIR          "G:/geodesic-master/build/eigen-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
