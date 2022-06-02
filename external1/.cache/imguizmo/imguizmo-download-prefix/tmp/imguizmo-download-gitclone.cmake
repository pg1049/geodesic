
if(NOT "G:/geodesic-master/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/imguizmo-download-gitinfo.txt" IS_NEWER_THAN "G:/geodesic-master/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/imguizmo-download-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'G:/geodesic-master/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/imguizmo-download-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "G:/geodesic-master/cmake/../external/imguizmo"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'G:/geodesic-master/cmake/../external/imguizmo'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" -c http.sslVerify=false clone --no-checkout --config "advice.detachedHead=false" --config "advice.detachedHead=false" "https://github.com/CedricGuillemet/ImGuizmo.git" "imguizmo"
    WORKING_DIRECTORY "G:/geodesic-master/cmake/../external"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/CedricGuillemet/ImGuizmo.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe" -c http.sslVerify=false checkout a23567269f6617342bcc112394bdad937b54b2d7 --
  WORKING_DIRECTORY "G:/geodesic-master/cmake/../external/imguizmo"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'a23567269f6617342bcc112394bdad937b54b2d7'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" -c http.sslVerify=false submodule update --recursive --init 
    WORKING_DIRECTORY "G:/geodesic-master/cmake/../external/imguizmo"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'G:/geodesic-master/cmake/../external/imguizmo'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "G:/geodesic-master/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/imguizmo-download-gitinfo.txt"
    "G:/geodesic-master/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/imguizmo-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'G:/geodesic-master/external/.cache/imguizmo/imguizmo-download-prefix/src/imguizmo-download-stamp/imguizmo-download-gitclone-lastrun.txt'")
endif()
