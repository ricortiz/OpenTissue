#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MESH_CLEAR_HALFEDGE_TAGS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MESH_CLEAR_HALFEDGE_TAGS_H
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
    template<typename mesh_type>
    void clear_halfedge_tags(std::shared_ptr<mesh_type> mesh, int tag_value = 0)
    {
      for(auto h : mesh->halfedges())
        h->m_tag = tag_value;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MESH_CLEAR_HALFEDGE_TAGS_H
#endif
