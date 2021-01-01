#ifndef OPENTISSUE_DYNAMICS_PSYS_UTIL_DIRECT_MESH_COUPLING_H
#define OPENTISSUE_DYNAMICS_PSYS_UTIL_DIRECT_MESH_COUPLING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <unordered_map>
#include <memory>

namespace OpenTissue
{
  namespace psys
  {

    template< typename types >
    class DirectMeshCoupling
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;

    public:

      typedef typename types::mesh_type        mesh_type;
      typedef typename mesh_type::vertex_type  vertex_type;

      typedef std::unordered_map<std::shared_ptr<vertex_type>,
                                 std::shared_ptr<particle_type>> particle_lut_type;

    protected:

      particle_lut_type           m_particle_lut;         ///< Internal data structure used to find correspond particle of vertex.
      std::shared_ptr<mesh_type>  m_mesh;                 ///< A pointer to the surface mesh.


    public:

      DirectMeshCoupling()
        : m_mesh(nullptr)
      {}

    public:

      std::shared_ptr<mesh_type>       mesh()       { return m_mesh; }
      std::shared_ptr<mesh_type> const mesh() const { return m_mesh; }

      std::shared_ptr<particle_type>       particle( std::shared_ptr<vertex_type> const v )       {  return m_particle_lut[v]; }
      std::shared_ptr<particle_type> const particle( std::shared_ptr<vertex_type> const v ) const {  return m_particle_lut[v]; }

      /**
       *
       * Adds binder support for AABB Tree.
       *
       *
       * @param  v
       * @return
       */
      std::shared_ptr<particle_type> operator()( std::shared_ptr<vertex_type> const v ) {  return this->particle(v); }

    public:

      bool empty() const {  return !m_mesh; }

      void clear()
      {
        m_particle_lut.clear();
        m_mesh = nullptr;
      }

      template<typename particle_system>
      void init(std::shared_ptr<particle_system> system, std::shared_ptr<mesh_type> mesh)
      {
        m_particle_lut.clear();
        system->clear();
        m_mesh = mesh;

        for(auto &v : mesh->vertices())
        {
          auto p = system->create_particle();
          p->bind(v->m_coord);
          m_particle_lut[v] = p;
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_UTIL_DIRECT_MESH_COUPLING_H
#endif
