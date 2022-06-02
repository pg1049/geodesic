# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(triangle-download NONE)

include(ExternalProject)
ExternalProject_Add(triangle-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/triangle.git;GIT_TAG;3ee6cac2230f0fe1413879574f741c7b6da11221
                    SOURCE_DIR          "G:/geodesic-master/cmake/../external/triangle"
                    BINARY_DIR          "G:/geodesic-master/build/triangle-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
