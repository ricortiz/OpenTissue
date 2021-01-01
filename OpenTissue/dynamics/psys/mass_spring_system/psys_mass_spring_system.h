#ifndef OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_MASS_SPRING_SYSTEM_H
#define OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_MASS_SPRING_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/iterator/indirect_iterator.hpp>
#include <boost/bind.hpp>

#include <list>
#include <memory>

namespace OpenTissue
{

  namespace psys
  {

    template<
      typename types
      , typename integrator_policy
    >
    class MassSpringSystem : public System<types>, public integrator_policy, std::enable_shared_from_this<MassSpringSystem<types,integrator_policy>>
    {
    public:

      typedef typename types::system_type                    system_type;
      typedef typename system_type::particle_iterator        particle_iterator;
      typedef typename system_type::const_particle_iterator  const_particle_iterator;

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;
      typedef typename types::constraint_type       constraint_type;
      typedef typename types::force_type            force_type;
      typedef typename types::geometry_holder_type  geometry_holder_type;
      typedef typename types::contact_point_type    contact_point_type;

    protected:

      unsigned int m_iterations;     ///< Maximum number of iterations of the constraint relaxation.
      bool         m_relaxation;     ///< Boolean flag indicating wheter contraint relaxation (shake and rattle) should be used.
      bool         m_projection;     ///< Boolean flag indicating wheter projection is used under relaxation to hande collisions with other objects.

    public:

      bool               & relaxation()       { return m_relaxation; }
      bool const         & relaxation() const { return m_relaxation; }
      bool               & projection()       { return m_projection; }
      bool const         & projection() const { return m_projection; }
      unsigned int       & iterations()       { return m_iterations; }
      unsigned int const & iterations() const { return m_iterations; }

    protected:

      typedef std::list<contact_point_type>               contact_point_container;
      typedef std::list<std::shared_ptr<force_type>>      force_ptr_container;
      typedef std::list<geometry_holder_type>             geometry_container;
      typedef std::list<std::shared_ptr<constraint_type>> constraint_ptr_container;

    protected:

      force_ptr_container        m_forces;       ///< A list of forces.
      constraint_ptr_container   m_constraints;  ///< A list of constraints.
      geometry_container         m_geometries;   ///< A list of geometries that this particle cluster
                                                 ///< should perform collision detection against.
    public:

      force_ptr_container            get_forces()            { return m_forces;      }
      const force_ptr_container      get_forces()      const { return m_forces;      }
      constraint_ptr_container       get_constraints()       { return m_constraints; }
      const constraint_ptr_container get_constraints() const { return m_constraints; }
      geometry_container             get_geometries()        { return m_geometries;  }
      const geometry_container       get_geometries()  const { return m_geometries;  }

      void clear(void)
      {
        m_forces.clear();
        m_constraints.clear();
        m_geometries.clear();
        system_type::clear();
      };

      void add_force(std::shared_ptr<force_type> F)              { F->connect(this->shared_from_this()); m_forces.push_back(F); }
      void remove_force(std::shared_ptr<force_type> F)           { m_forces.remove(F); F->disconnect();      }

      void add_constraint(std::shared_ptr<constraint_type> C)    { C->connect(this->shared_from_this()); m_constraints.push_back(C); }
      void remove_constraint(std::shared_ptr<constraint_type> C) { m_constraints.remove(C); C->disconnect();      }

      template<typename geometry_type>
      void add_geometry(std::shared_ptr<geometry_type> G)
      {
        geometry_holder_type holder;
        holder.set(G);
        holder.connect(this->shared_from_this());
        m_geometries.push_back(holder);
      }

      template<typename geometry_type>
      void remove_geometry(std::shared_ptr<geometry_type> G)
      {
        geometry_holder_type holder;
        holder.set(G);
        m_geometries.remove(holder);
      }

    public:

      MassSpringSystem(void)
        : m_iterations(10)
        , m_relaxation(true)
        , m_projection(true)
      {}

      ~MassSpringSystem(){ clear(); }

    public:

      void run(real_type timestep)
      {
        assert(timestep>0 || !"MassSpringSystem::run(): Non-positive time-step");
        integrator_policy::integrate ( *this, timestep ); //--- from integrator policy
        do_relaxation();
        this->time() += timestep;
      }

    protected:

      void do_relaxation()
      {
        if(!m_relaxation)
          return;

        for(unsigned int i=0;i<m_iterations;++i)
        {
          for(auto c : m_constraints)
            c->satisfy();

          do_projection();
        }
      }

      void do_projection()
      {
        if(!m_projection)
          return;

        contact_point_container contacts;

        for(auto &g : m_geometries)
          g.dispatch( *this, contacts );

        for(auto &cp : contacts)
        {
          if(cp.m_A0 && !cp.m_A1 && !cp.m_A2 && !cp.m_B0 && !cp.m_B1 && !cp.m_B2)
          {
            cp.m_A0->position() += cp.m_n*cp.m_distance;
          }

          if(cp.m_A0 && cp.m_A1 && !cp.m_A2 && cp.m_B0 && cp.m_B1 && cp.m_B2)
          {
            //--- Hack, I have not really thought about what to do?
            cp.m_A0->position() = cp.m_A0->old_position();
            cp.m_A1->position() = cp.m_A1->old_position();

            cp.m_A0->velocity().clear();
            cp.m_A1->velocity().clear();

            cp.m_B0->position() = cp.m_B0->old_position();
            cp.m_B1->position() = cp.m_B1->old_position();
            cp.m_B2->position() = cp.m_B2->old_position();

            cp.m_B0->velocity().clear();
            cp.m_B1->velocity().clear();
            cp.m_B2->velocity().clear();
          }

        }
      }

    public:

      void compute_accelerations()
      {
        for(auto &p : this->m_particles)
          p->acceleration() = p->force() * p->inv_mass();
      }

      void compute_forces()
      {
        for(auto &p : this->m_particles)
          p->force().clear();

        for(auto f : m_forces)
        {
          f->apply();
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_MASS_SPRING_SYSTEM_PSYS_MASS_SPRING_SYSTEM_H
#endif
