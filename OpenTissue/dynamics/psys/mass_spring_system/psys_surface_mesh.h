#ifndef OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_SURFACE_MESH_H
#define OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_SURFACE_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/constraints/psys_stick.h>
#include <OpenTissue/dynamics/psys/forces/psys_spring.h>
#include <OpenTissue/dynamics/psys/mass_spring_system/psys_mass_spring_system.h>

#include <list>
#include <unordered_map>
#include <memory>

namespace OpenTissue
{
  namespace psys
  {

    /**
    * Surface Mesh.
    *
    * Takes a mesh and builds a particle system from it.
    */
    template<
        typename types
      , typename integrator_policy
    >
    class SurfaceMesh : public MassSpringSystem<types,integrator_policy>, std::enable_shared_from_this<SurfaceMesh<types,integrator_policy>>
    {
    public:

      typedef MassSpringSystem<types,integrator_policy> base_class;

    public:

      typedef typename types::math_types         math_types;
      typedef typename math_types::real_type     real_type;
      typedef typename math_types::vector3_type  vector3_type;
      typedef typename types::particle_type      particle_type;
      typedef Stick<types>                       stick_type;
      typedef Spring<types>                      spring_type;
      typedef typename types::coupling_type      coupling_type;
      typedef typename types::mesh_type          mesh_type;

    protected:

      typedef typename mesh_type::vertex_type                                       vertex_type;
      typedef typename mesh_type::vertex_iterator                                   vertex_iterator;
      typedef typename mesh_type::vertex_halfedge_circulator                        vertex_halfedge_circulator;

      typedef std::list<std::shared_ptr<stick_type>>                                stick_container;
      typedef std::unordered_map<std::shared_ptr<particle_type>, stick_container >  stick_lut_type;

      typedef std::list<std::shared_ptr<spring_type>>                               spring_container;
      typedef std::unordered_map<std::shared_ptr<particle_type>, spring_container > spring_lut_type;

    public:

      std::shared_ptr<coupling_type> m_coupling;             ///< Internal data structure used to find correspond particle of vertex.
      int                            m_rigidty;              ///< Constant used to determine how rigid a surface should be.
      stick_container                m_sticks;               ///< Internal data structure used to store all stick constraints.
      stick_lut_type                 m_stick_lut;            ///< Internal datas tructure to record stick connections.
      spring_container               m_springs;              ///< Internal data structure used to store all spring constraints.
      spring_lut_type                m_spring_lut;           ///< Internal datas tructure to record spring connections.

    public:

      int                  & rigidity()                       { return m_rigidty;  }
      int const            & rigidity()  const                { return m_rigidty;  }
      std::shared_ptr<coupling_type>        coupling()       { return m_coupling; }
      std::shared_ptr<coupling_type>  const coupling() const { return m_coupling; }

    protected:

      bool exist_stick(std::shared_ptr<particle_type> A, std::shared_ptr<particle_type> B)
      {

        auto sticksA = m_stick_lut[A];
        auto sticksB = m_stick_lut[B];
        if(sticksA.empty() || sticksB.empty())
          return false;
        {
          for(auto s : sticksA)
          {
            if( (s->A() == A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        {
          for(auto s : sticksB)
          {
            if( (s->A()==A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        return false;
      }

      bool exist_spring(std::shared_ptr<particle_type> A, std::shared_ptr<particle_type> B)
      {
        auto springsA = m_spring_lut[A];
        auto springsB = m_spring_lut[B];
        if(springsA.empty() || springsB.empty())
          return false;
        {
          for(auto s : springsA)
          {
            if( (s->A() == A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        {
          for(auto s : springsB)
          {
            if( (s->A()==A && s->B()==B) || (s->B()==A && s->A()==B) )
              return true;
          }
        }
        return false;
      }


      void add_stick(std::shared_ptr<particle_type> A, std::shared_ptr<particle_type> B)
      {
        if(A==B)
          return;

        auto s = std::make_shared<stick_type>();
        this->add_constraint( s );
        s->init(A,B);
        m_stick_lut[A].push_back(s);
        m_stick_lut[B].push_back(s);
        m_sticks.push_back(s);
      }

      void add_spring(std::shared_ptr<particle_type> A, std::shared_ptr<particle_type> B)
      {
        if(A==B)
          return;

        auto s = std::make_shared<spring_type>();
        this->add_force( s );
        s->init(A,B);
        m_spring_lut[A].push_back(s);
        m_spring_lut[B].push_back(s);
        m_springs.push_back(s);
      }


      template<typename vertex_iterator_container>
      void traverse(
          vertex_iterator root
        , vertex_iterator from
        , bool create_sticks
        , bool create_springs
        , int dist
        , vertex_iterator_container & visited
      )
      {
        if( dist >= m_rigidty )
          return;

        visited.push_back(from);
        from->m_tag = 1;

        vertex_halfedge_circulator h(*from),hend;
        for(;h!=hend;++h)
        {
          auto to = h->get_destination_iterator();
          if( to->m_tag == 0 )
          {
            auto A = m_coupling->particle(*root);
            auto B = m_coupling->particle(*to);

            if(create_sticks && !exist_stick(A,B))
              add_stick(A,B);

            if(create_springs && !exist_spring(A,B))
              add_spring(A,B);

            traverse( root, to, create_sticks, create_springs, dist+1, visited);
          }
        }
      }

    public:

      SurfaceMesh()
        : m_rigidty(2)
        , m_sticks()
        , m_stick_lut()
        , m_springs()
        , m_spring_lut()
      {
        m_coupling = std::make_shared<coupling_type>();
      }

      virtual ~SurfaceMesh() {}

    public:

      virtual void clear()
      {
        base_class::clear();

        m_coupling->clear();

        m_sticks.clear();
        m_springs.clear();
        m_stick_lut.clear();
        m_spring_lut.clear();
      }

      virtual void init(std::shared_ptr<mesh_type> mesh, bool create_sticks, bool create_springs)
      {
        this->clear();

        m_coupling->init(this->shared_from_this(), mesh);

        mesh::clear_vertex_tags(m_coupling->mesh());

        vertex_iterator end   = m_coupling->mesh().vertex_end();
        vertex_iterator v     = m_coupling->mesh().vertex_begin();
        for(;v!=end;++v)
        {
          std::list<vertex_iterator> visited;

          traverse( v, v, create_sticks, create_springs, 0, visited);

          for(typename std::list<vertex_iterator>::iterator w = visited.begin();w!=visited.end();++w)
            (*w)->m_tag = 0;
        }

        m_stick_lut.clear();
        m_spring_lut.clear();
      }

    };
  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_SURFACE_MESH_H
#endif
