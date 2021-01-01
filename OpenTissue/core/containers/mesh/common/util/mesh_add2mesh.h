#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_ADD2MESH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_ADD2MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <map>
#include <memory>
#include <list>

namespace OpenTissue
{
  namespace mesh
  {


    /**
     * Add Mesh to Another Mesh.
     *
     * @param mesh0    A reference to the input mesh.
     * @param mesh1    Upon return the geometry of mesh mesh0 will have been added to this mesh.
     *
     * @return     If operation is succesful then the return value is
     *             true otherwise it is false.
     */
    template<typename mesh_type>
    bool add2mesh(
        std::shared_ptr<mesh_type> const mesh0
      , std::shared_ptr<mesh_type> mesh1
      )
    {
      typedef typename mesh_type::vertex_handle                     vertex_handle;
      typedef typename mesh_type::face_handle                       face_handle;
      typedef typename mesh_type::vertex_iterator                   vertex_iterator;
      typedef typename mesh_type::face_iterator                     face_iterator;
      typedef typename mesh_type::const_vertex_iterator             const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator               const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator      const_face_vertex_circulator;
      typedef typename mesh_type::vertex_traits                     vertex_traits;
      typedef typename mesh_type::face_traits                       face_traits;
      typedef typename mesh_type::index_type                        index_type;

      typedef std::map<index_type,vertex_handle>                    vh_lut_type;

      vh_lut_type vh_lut;

      for(auto vertex : mesh0->vertices())
      {
        vertex_handle h = mesh1.add_vertex();
        assert(!h.is_null() || !"add2mesh(): Internal error, Could not create vertex in output mesh");

        vh_lut[vertex->get_handle().get_idx()] = h;

        auto new_vertex = *mesh1->get_vertex_iterator(h);
        auto vertex_trait = std::static_pointer_cast<vertex_traits>(new_vertex);
        *new_vertex = *std::static_pointer_cast<vertex_traits const>(v);
      }

      for(auto f : mesh0->faces())
      {
        std::list<vertex_handle> handles;
        for(auto vertex : const_face_vertex_circulator(f))
        {
          vertex_handle h = vh_lut[vertex->get_handle().get_idx()];
          assert(!h.is_null() || !"add2mesh(): Internal error, could not find vertices in output mesh");
          handles.push_back(h);
        }

        face_handle h = mesh1->add_face(handles.begin(),handles.end());
        assert(!h.is_null() || !"add2mesh(): Internal error, Could not create face in output mesh");

        auto new_face = *mesh1->get_face_iterator(h);
        auto face_trait = std::static_pointer_cast<face_traits>(new_face);

        *face_trait = *std::static_pointer_cast<face_traits const>(f);
      }
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_ADD2MESH_H
#endif
