#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_SYSTEM_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <vector>

#include <memory>

namespace OpenTissue
{
  namespace psys
  {
    /**
     * This is the ``core'' data structure for particles. It provides iterator
     * capabilities. Particle lookup based on indices and time-management of
     * the entire particle system.
     *
     * Also there is simple support functionality for deriving AABB information
     * of the particle system (see min and max methods).
     */
    template<typename types >
    class System : public std::enable_shared_from_this<System<types>>
    {
    public:

      typedef typename types::math_types                            math_types;
      typedef typename math_types::real_type                        real_type;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename types::particle_type                         particle_type;

      typedef          std::vector<std::shared_ptr<particle_type>>  particle_container;
      typedef typename particle_container::iterator                 particle_iterator;
      typedef typename particle_container::const_iterator           const_particle_iterator;


    protected:

      real_type            m_time;         ///< Current time.
      particle_container   m_particles;    ///< A vector of all particles in the cluster.

    public:

      real_type       & time()       { return m_time; }
      real_type const & time() const { return m_time; }

    public:

      System()
        : m_time(0.0)
      {}

      virtual ~System(){  };

    public:

      vector3_type min_coord()
      {
        // TODO: Is it possible to add some lazy evalutation to this? Caching
        // previous computed value and only re-compute it if particles have
        // changed?
        vector3_type min_coord = vector3_type( math::detail::highest<real_type>(),math::detail::highest<real_type>(),math::detail::highest<real_type>() );
        for(auto p : m_particles)
          min_coord = std::min( min_coord, p->position() );
      }

      vector3_type max_coord()
      {
        // TODO: Is it possible to add some lazy evalutation to this? Caching
        // previous computed value and only re-compute it if particles have
        // changed?
        vector3_type max_coord = vector3_type( math::detail::lowest<real_type>(),math::detail::lowest<real_type>(),math::detail::lowest<real_type>() );
        for(auto p : m_particles)
          max_coord = std::max( max_coord, p->position() );
      }

    public:

      void clear()
      {
        m_particles.clear();
      }

      std::shared_ptr<particle_type> create_particle()
      {
        auto p = std::make_shared<particle_type>();
        p->connect(this->shared_from_this());
        m_particles.push_back(p);
        return p;
      }

      std::shared_ptr<particle_type> create_particles(const size_t n)
      {
        for(auto i = 0; i < n; ++i)
        {
          this->create_particle();
        }
      }

      void erase(std::shared_ptr<particle_type> p)
      {
        auto p_it = std::find(m_particles.begin(), m_particles.end(), p);
        if(p_it != m_particles.end())
        {
          *p_it->disconnect();
          m_particles.erase(p);
        }
      }

      particle_iterator       & operator()(size_t idx)       { return m_particles.begin() + idx; }
      const_particle_iterator & operator()(size_t idx) const { return m_particles.begin() + idx; }

      particle_iterator       begin()       { return m_particles.begin(); }
      particle_iterator       end()         { return m_particles.end(); }
      const_particle_iterator begin() const { return m_particles.begin(); }
      const_particle_iterator end()   const { return m_particles.end();   }

      std::size_t particles_size() const { return m_particles.size(); }

      particle_container & particles() { return m_particles; }
      particle_container const & particles() const { return m_particles; }

    };

  } // namespace psys
} // namespace OpenTissue

template<typename Types>
typename OpenTissue::psys::System<Types>::particle_iterator       begin(OpenTissue::psys::System<Types> &system)        { return system.begin(); }
template<typename Types>
typename OpenTissue::psys::System<Types>::particle_iterator       end(OpenTissue::psys::System<Types> &system)          { return system.end(); }
template<typename Types>
typename OpenTissue::psys::System<Types>::const_particle_iterator begin(const OpenTissue::psys::System<Types> &system)  { return system.begin(); }
template<typename Types>
typename OpenTissue::psys::System<Types>::const_particle_iterator end(const OpenTissue::psys::System<Types> &system)    { return system.end(); }

// OPENTISSUE_DYNAMICS_PSYS_PSYS_SYSTEM_H
#endif
