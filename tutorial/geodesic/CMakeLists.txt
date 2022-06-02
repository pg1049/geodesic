get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

file(GLOB sources
*.h
*.cpp
*.c
)
file(GLOB sdk
sdk/*.h
sdk/*.cpp
sdk/*.c
)

add_executable(${PROJECT_NAME}_bin ${sources} ${sdk})
source_group("geodesic_algorithm" FILES ${sdk})
target_link_libraries(${PROJECT_NAME}_bin igl::core igl::opengl igl::opengl_glfw   igl::opengl_glfw_imgui  tutorials)
