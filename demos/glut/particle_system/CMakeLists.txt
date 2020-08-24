add_executable(particle_system src/application.cpp)

target_link_libraries(particle_system
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
  TARGETS particle_system
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
