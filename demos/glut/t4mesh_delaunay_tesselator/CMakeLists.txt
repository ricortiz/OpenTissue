add_executable(t4mesh_delaunay_tesselator src/application.cpp)

target_link_libraries(t4mesh_delaunay_tesselator
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    Qhull::libqhull
    OpenTissue
)

install(
  TARGETS t4mesh_delaunay_tesselator
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
