
if(NOT "G:/geodesic/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/libigl-imgui-download-gitinfo.txt" IS_NEWER_THAN "G:/geodesic/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/libigl-imgui-download-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'G:/geodesic/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/libigl-imgui-download-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "G:/geodesic/cmake/../external/libigl-imgui"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'G:/geodesic/cmake/../external/libigl-imgui'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" -c http.sslVerify=false clone --no-checkout --config "advice.detachedHead=false" --config "advice.detachedHead=false" "https://github.com/libigl/libigl-imgui.git" "libigl-imgui"
    WORKING_DIRECTORY "G:/geodesic/cmake/../external"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/libigl/libigl-imgui.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe" -c http.sslVerify=false checkout 7e1053e750b0f4c129b046f4e455243cb7f804f3 --
  WORKING_DIRECTORY "G:/geodesic/cmake/../external/libigl-imgui"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '7e1053e750b0f4c129b046f4e455243cb7f804f3'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe" -c http.sslVerify=false submodule update --recursive --init 
    WORKING_DIRECTORY "G:/geodesic/cmake/../external/libigl-imgui"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'G:/geodesic/cmake/../external/libigl-imgui'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "G:/geodesic/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/libigl-imgui-download-gitinfo.txt"
    "G:/geodesic/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/libigl-imgui-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'G:/geodesic/external/.cache/libigl-imgui/libigl-imgui-download-prefix/src/libigl-imgui-download-stamp/libigl-imgui-download-gitclone-lastrun.txt'")
endif()

