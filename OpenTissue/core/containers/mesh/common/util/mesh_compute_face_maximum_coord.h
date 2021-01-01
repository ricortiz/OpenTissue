#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_MAXIMUM_COORD_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_MAXIMUM_COORD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <memory>

namespace OpenTissue
{
  namespace mesh
  {

    template<typename face_type,typename vector3_type>
    void compute_face_maximum_coord(std::shared_ptr<face_type> const f, vector3_type & max_coord)
    {
      typedef typename face_type::mesh_type               mesh_type;
      typedef typename mesh_type::face_vertex_circulator  face_vertex_circulator;
      typedef std::numeric_limits<typename vector3_type::value_type> limits;

      max_coord = limits::min();
      for(auto v: face_vertex_circulator(f))
        max_coord = std::max(max_coord,v->m_coord);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_FACE_MAXIMUM_COORD_H
#endif
