//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/utility/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

#include <OpenTissue/dynamics/psys/psys.h>
#include <OpenTissue/core/math/math_basic_types.h>

#include <memory>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
public:

  typedef OpenTissue::math::BasicMathTypes<double,size_t>      math_types;
  typedef OpenTissue::psys::Types<math_types>            types;
  //typedef OpenTissue::psys::Types<double,OpenTissue::psys::EulerIntegrator> types;

  typedef math_types::real_type                           real_type;
  typedef math_types::vector3_type                        vector3_type;
  typedef math_types::matrix3x3_type                      matrix3x3_type;

protected:

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.

  static unsigned char const ONE_KEY   = '1';
  static unsigned char const TWO_KEY   = '2';
  static unsigned char const THREE_KEY = '3';
  static unsigned char const FOUR_KEY  = '4';
  static unsigned char const FIVE_KEY  = '5';
  static unsigned char const SIX_KEY   = '6';
  static unsigned char const SEVEN_KEY = '7';

  // Systems
  std::shared_ptr<types::mass_spring_system_type>           m_mass_spring;
  std::shared_ptr<types::cloth_system_type>                 m_cloth;
  std::shared_ptr<types::surface_system_type>               m_surface;
  // Geometries
  // Forces
  // Structures
  // Constraints

  real_type                                m_timestep;
  bool                                     m_simulate_on;  ///< Boolean flag used to toggle simulation on/off.

protected:

  void cleanup(  )
  {
    m_surface->clear();
    m_cloth->clear();
    m_mass_spring->clear();
  }

  std::shared_ptr<types::cloth_system_type> cloth_setup(
       bool create_sticks = true
    ,  bool create_springs = false
    , unsigned int rigidity = 2
    )
  {
    cleanup();

    auto cloth_system = std::make_shared<types::cloth_system_type>();
    cloth_system->rigidity() = rigidity;
    cloth_system->init(1.75,1.75,20,20, create_sticks, create_springs);
    cloth_system->iterations() = 10;
    cloth_system->relaxation() = true;
    cloth_system->projection() = true;

    //--- Initialize geometries... ---------------------------------------------------
    auto plane = std::make_shared<types::plane_type>();
    plane->n() = vector3_type(0,0,1);
    plane->w() = -5;
    cloth_system->add_geometry(plane);

    //--- Initialize forces ----------------------------------------------------------
    auto gravity = std::make_shared<types::gravity_type>();
    auto viscosity = std::make_shared<types::viscosity_type>();
    gravity->gravity() = 9.8;
    viscosity->viscosity() = 0.5;
    cloth_system->add_force(gravity);
    cloth_system->add_force(viscosity);

    //--- Initialize constraints -----------------------------------------------------
    auto box = std::make_shared<types::box_constraint_type>();
    box->aabb().min() = vector3_type(-3,-3,-6);
    box->aabb().max() = vector3_type( 3, 3, 6);
    cloth_system->add_constraint(box);

    if(create_springs)
      m_timestep = 0.001;
    else
      m_timestep = 0.01;

    return cloth_system;
  }

  std::shared_ptr<types::surface_system_type> surface_setup(
      std::string const & filename
    , bool create_sticks = true
    ,  bool create_springs = false
    , unsigned int rigidity = 3
    , bool with_gravity = true
    )
  {
    cleanup();

    std::string data_path = opentissue_path;
    std::string meshfile = data_path + filename;

    auto mesh = std::make_shared<types::mesh_type>();
    OpenTissue::mesh::obj_read(meshfile,mesh);
    OpenTissue::mesh::make_unit(mesh);
    OpenTissue::mesh::translate(mesh, vector3_type( -0.5, -0.5, 5));

    auto surface_system = std::make_shared<types::surface_system_type>();
    surface_system->rigidity() = rigidity;
    surface_system->init(mesh, create_sticks, create_springs);
    surface_system->iterations() = 10;
    surface_system->relaxation() = true;
    surface_system->projection() = true;

    //--- Initialize geometries... ---------------------------------------------------
    auto sphere = std::make_shared<types::sphere_type>();
    sphere->radius(4);
    sphere->center( vector3_type(0,0,-8) );

    auto plane = std::make_shared<types::plane_type>();
    plane->n() = vector3_type(0,0,1);
    plane->w() = -5;

    surface_system->add_geometry(sphere);
    surface_system->add_geometry(plane);

    //--- Initialize forces ----------------------------------------------------------
    auto viscosity = std::make_shared<types::viscosity_type>();
    viscosity->viscosity() = 0.5;
    surface_system->add_force(viscosity);
    if(with_gravity)
    {
      auto gravity = std::make_shared<types::gravity_type>();
      gravity->gravity() = 9.8;
      surface_system->add_force(gravity);
    }

    //--- Initialize constraints -----------------------------------------------------
    auto box = std::make_shared<types::box_constraint_type>();
    box->aabb().min() = vector3_type(-3,-3,-6);
    box->aabb().max() = vector3_type( 3, 3, 6);
    surface_system->add_constraint(box);

    if(create_springs)
      m_timestep = 0.001;
    else
      m_timestep = 0.01;

    return surface_system;
  }

public:

  Application(){}

public:

  char const * do_get_title() const { return "Particle System Demo Application"; }

  void do_display()
  {
    OpenTissue::gl::ColorPicker( 0.1, 0.4, 0.8 );
    OpenTissue::gl::DrawSphere( m_sphere, true );
    OpenTissue::gl::DrawPlane( m_plane, true );

    OpenTissue::gl::ColorPicker( 0.1, 0.8, 0.4 );

    if(!m_surface->coupling()->empty())
    {
      OpenTissue::gl::DrawMesh( m_surface->coupling()->mesh() , GL_LINE_LOOP );
    }

    if(!m_cloth->coupling()->empty())
      OpenTissue::gl::DrawMesh( m_cloth->coupling()->mesh() , GL_LINE_LOOP );

    OpenTissue::gl::ColorPicker(0.8, 0.4, 0.1);
    OpenTissue::gl::DrawAABB(m_box->aabb(), true);
    OpenTissue::gl::ColorPicker( 0.8, 0.4, 0.1 );
    OpenTissue::gl::DrawPoint(m_pin->pin_position());

    OpenTissue::gl::ColorPicker( 0.8, 0.8, 0.1 );
    //OpenTissue::aabb_tree_debug_draw(m_aabb_tree);

    //OpenTissue::sdf::debug_draw_sampling( m_sdf_geometry );
    //OpenTissue::sdf::debug_draw_bvh( m_sdf_geometry, 0);
    OpenTissue::gl::DrawMesh(m_sdf_geometry->m_mesh , GL_LINE_LOOP);

    OpenTissue::gl::ColorPicker( 0.8, 0.1, 0.8 );
    for(auto p : *m_mass_spring)
    {
      OpenTissue::gl::DrawPoint(p->position());
    }

  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    switch ( choice )
    {
    case 't':       run();   break;
    case 's':
      m_simulate_on = !m_simulate_on;
      std::cout << " auto simulate = " << m_simulate_on << std::endl;
      break;
    case '1':
      {
        m_surface = surface_setup("/demos/data/obj/box.obj");

        m_action[ONE_KEY] = true;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '2':
      {
        m_surface = surface_setup("/demos/data/obj/box.obj");

        auto pin = std::make_shared<types::pin_constraint_type>();
        pin->init(*m_surface->begin()); // pin the first particle
        m_surface->add_constraint(pin);

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = true;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '3':
      {
        m_surface = surface_setup("/demos/data/obj/box.obj", false);

        auto force_grid = std::make_shared<types::grid_force_field_type::grid_type>();
        force_grid->create( vector3_type(-5,-5,-5), vector3_type(5,5,5), 32, 32, 32);
        OpenTissue::psys::compute_perlin_noise_force_field(force_grid);//--- just a compile test:-)
        OpenTissue::psys::compute_random_force_field(force_grid, 10.0);

        auto force_field = std::make_shared<types::grid_force_field_type>();
        force_field->init(force_grid);
        m_surface->add_force(force_field);

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = true;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '4':
      {
        m_surface = surface_setup("/demos/data/obj/cylinder.obj", false, true, 1);

        auto pressure = std::make_shared<types::pressure_soft_body_type>();
        pressure->set_initial_pressure(5000.0);
        pressure->init(m_surface->coupling());
        m_surface->add_force(pressure);

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = true;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '5':
      {
        m_cloth = cloth_setup();

        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/box.obj";

        auto mesh = std::make_shared<types::mesh_type>();
        OpenTissue::mesh::obj_read(meshfile, mesh);
        OpenTissue::mesh::make_unit(mesh);
        OpenTissue::mesh::translate(mesh, vector3_type( 0, 0, -5));

        auto sdf_geometry = std::make_shared<types::sdf_geometry_type>();
        OpenTissue::sdf::semiauto_init_geometry( mesh, 0.01, true, sdf_geometry);
        m_cloth->add_geometry(sdf_geometry);

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = true;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '6':
      {
        cleanup();
        //--- Initialize geometries... ---------------------------------------------------
        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/propella.obj";

        auto mesh = std::make_shared<types::mesh_type>();
        OpenTissue::mesh::obj_read(meshfile, mesh);
        OpenTissue::mesh::make_unit(mesh);
        OpenTissue::mesh::uniform_scale(mesh, 2.0);

        m_surface = std::make_shared<types::surface_system_type>();
        m_surface->rigidity() = 2;
        m_surface->init(mesh, true, false);
        m_surface->iterations() = 10;
        m_surface->relaxation() = true;
        m_surface->projection() = true;

        auto aabb_tree = std::make_shared<types::aabb_tree_type>();
        OpenTissue::aabb_tree::init(m_surface->coupling()->mesh(), aabb_tree, m_surface->coupling());

        m_mass_spring = std::make_shared<types::mass_spring_system_type>();

        m_mass_spring->add_geometry(aabb_tree);

        auto plane = std::make_shared<types::plane_type>();
        plane->n() = vector3_type(0,0,1);
        plane->w() = -5;
        m_mass_spring->add_geometry(plane);

        //--- Initialize constraints -----------------------------------------------------
        auto box = std::make_shared<types::box_constraint_type>();
        box->aabb().min() = vector3_type(-3,-3,-6);
        box->aabb().max() = vector3_type( 3, 3, 6);
        m_mass_spring->add_constraint(box);

        m_timestep = 0.01;

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = true;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '7':
      {
        cleanup();
        //--- Initialize geometries... ---------------------------------------------------
        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/propella.obj";

        auto mesh = std::make_shared<types::mesh_type>();
        OpenTissue::mesh::obj_read(meshfile,mesh);
        OpenTissue::mesh::make_unit(mesh);
        OpenTissue::mesh::uniform_scale(mesh, 2.0 );

        m_surface = std::make_shared<types::surface_system_type>();
        m_surface->rigidity() = 2;
        m_surface->init(mesh, true, false);
        m_surface->iterations() = 10;
        m_surface->relaxation() = true;
        m_surface->projection() = true;

        auto aabb_tree = std::make_shared<types::aabb_tree_type>();
        OpenTissue::aabb_tree::init(m_surface->coupling()->mesh(), aabb_tree, m_surface->coupling());

        m_surface->add_geometry(aabb_tree);

        auto sphere = std::make_shared<types::sphere_type>();
        sphere->radius(4);
        sphere->center( vector3_type(0,0,-8) );
        m_surface->add_geometry(sphere);

        auto plane = std::make_shared<types::plane_type>();
        plane->n() = vector3_type(0,0,1);
        plane->w() = -5;
        m_surface->add_geometry(plane);

        auto gravity = std::make_shared<types::gravity_type>(9.8);
        auto viscosity = std::make_shared<types::viscosity_type>(0.5);
        m_surface->add_force(gravity);
        m_surface->add_force(viscosity);

        m_timestep = 0.01;

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = true;
      }
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry("Toggle auto simulate                  [s]", 's' );
    glutAddMenuEntry("Single step                           [t]", 't' );
    glutAddMenuEntry("Stick Box setup                       [1]", '1' );
    glutAddMenuEntry("Pin constraint                        [2]", '2' );
    glutAddMenuEntry("Random force field                    [3]", '3' );
    glutAddMenuEntry("Pressure forces                       [4]", '4' );
    glutAddMenuEntry("cloth vs. signed distance field test  [5]", '5' );
    glutAddMenuEntry("particle sys vs. particle sys         [6]", '6' );
    glutAddMenuEntry("particle self-collision               [7]", '7' );

    glutSetMenu( main_menu );
    glutAddSubMenu( "particle system", controls );
  }

  void do_init()
  {
    m_simulate_on = false;
  }

  void do_run()
  {
    if(m_simulate_on)
    {
      if(m_action[SIX_KEY])
      {
        if(m_mass_spring->particles_size() < 50)
        {
          auto p = m_mass_spring->create_particle();
          OpenTissue::math::random(p->position(),-1,1);
          p->position()(0) = -2;
          p->velocity() = vector3_type(1.0,0,0);
          p->old_position() = p->position() - p->velocity()*m_timestep;

        }
        m_mass_spring->run(m_timestep);
      }
      else
      {
        m_surface->run(m_timestep);
        m_cloth->run(m_timestep);
      }
    }
  }

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
