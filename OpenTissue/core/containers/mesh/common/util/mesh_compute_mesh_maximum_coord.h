#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MAXIMUM_COORD_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MAXIMUM_COORD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mesh
  {

    template<typename mesh_type, typename vector3_type>
    void compute_mesh_maximum_coord(std::shared_ptr<mesh_type> const mesh, vector3_type & max_coord)
    {
      using value_type = typename vector3_type::value_type;
      assert(mesh->size_vertices()>0 || !"mesh did not have any vertices");

      auto constexpr inf = std::numeric_limits<value_type>::min();
      max_coord = {inf, inf, inf};
      for(auto v : mesh->vertices())
        max_coord = max(max_coord, v->m_coord);
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COMPUTE_MESH_MAXIMUM_COORD_H
#endif
