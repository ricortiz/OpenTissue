add_executable(multibody_cows src/application.cpp)

target_link_libraries(multibody_cows
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
    Boost::headers
)

install(
  TARGETS multibody_cows
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
