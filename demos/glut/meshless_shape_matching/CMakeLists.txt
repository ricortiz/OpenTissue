add_executable(meshless_shape_matching src/application.cpp)

target_link_libraries(meshless_shape_matching
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
    Boost::headers
    Qhull::libqhull
)

install(
  TARGETS meshless_shape_matching
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
