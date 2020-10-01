add_executable(chanvese_segmentation src/application.cpp)

target_link_libraries(chanvese_segmentation
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    Boost::headers
    OpenTissue
    OpenTissueGraphics
)

target_compile_definitions(chanvese_segmentation 
  PRIVATE
   OT_INJECT_MAIN
)
