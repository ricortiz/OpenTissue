#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_COMPUTE_MEAN_VERTEX_NORMALS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_COMPUTE_MEAN_VERTEX_NORMALS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>
#include <cmath>
#include <vector>
#include <memory>

namespace OpenTissue
{
  namespace mesh
  {

    template<typename mesh_type>
    void compute_mean_vertex_normals(std::shared_ptr<mesh_type> & mesh)
    {
      typedef typename mesh_type::math_types              math_types;
      typedef typename math_types::value_traits           value_traits;
      typedef typename math_types::vector3_type           vector3_type;
      typedef typename math_types::real_type              real_type;

      typedef typename mesh_type::vertex_iterator         vertex_iterator;
      typedef typename mesh_type::face_iterator           face_iterator;
      typedef typename mesh_type::face_vertex_circulator  face_vertex_circulator;
      {
        for(auto v : mesh->vertices())
          v->m_normal.clear();
      }
      {
        for(auto f : mesh->faces())
        {
          face_vertex_circulator i (f),vend;
          face_vertex_circulator j (f);
          face_vertex_circulator k (f);
          ++j; ++k;++k;
          for(;i!=vend;++i,++j,++k)
          {
            vector3_type u0 = normalize(j->m_coord - i->m_coord);
            vector3_type u1 = normalize(k->m_coord - j->m_coord);
            vector3_type n  = normalize(u0 % u1);
            j->m_normal += n;
          }
        }
      }
      {
        for(auto v : mesh->vertices())
          v->m_normal = normalize(v->m_normal);
      }
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_COMPUTE_MEAN_VERTEX_NORMALS_H
#endif
