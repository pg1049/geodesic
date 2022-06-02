# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 3.1)

project(imguizmo-download NONE)

include(ExternalProject)
ExternalProject_Add(imguizmo-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/CedricGuillemet/ImGuizmo.git;GIT_TAG;a23567269f6617342bcc112394bdad937b54b2d7
                    SOURCE_DIR          "G:/geodesic-master/cmake/../external/imguizmo"
                    BINARY_DIR          "G:/geodesic-master/build/imguizmo-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
