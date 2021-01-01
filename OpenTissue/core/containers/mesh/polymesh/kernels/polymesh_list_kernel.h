#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_KERNELS_POLYMESH_LIST_KERNEL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_KERNELS_POLYMESH_LIST_KERNEL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

#include <boost/optional.hpp>
#include <boost/none.hpp>

#include <list>
#include <vector>
#include <cassert>
#include <memory>
#include <unordered_map>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * A PolyMesh kernel is the basic storage of a polygon mesh. It is
    * responsible for defining types such as vertices, edges and faces,
    * but also iterators and handles.
    *
    * Iterators should at very least be forward traversal iterators or at best
    * a bidirectional iterator.
    *
    * Handles is an identifier like concept. It uniquely identifies are vertex,
    * edge or face. Handles should be persistent, which means that old handles
    * are still valid upon deletion or insertion of new vertices, edges or faces.
    * This is different from iterators which may all become invalid upon insertion
    * or deletion. In fact a handle is more or less identical to an unique index.
    *
    * Handles are thus very efficient for copying and identifying features in
    * the mesh.
    *
    * The kernel can be seen as a multi index table, users can retrieve information
    * based on either iterators or handles, or make conversions between the two.
    *
    * Notice, when deleting an entity identified by a handle, one can query if the
    * handle is still valid.
    */
    template<
        typename V
      , typename H
      , typename E
      , typename F
    >
    class PolyMeshListKernel
    {
    public:

      typedef V   vertex_type;
      typedef H   halfedge_type;
      typedef E   edge_type;
      typedef F   face_type;

      typedef PolyMeshListKernel<V,H,E,F> kernel_type;

    public:

      typedef std::size_t                                                        index_type;
      typedef typename std::list<std::shared_ptr<vertex_type>>::size_type        size_type;
      typedef typename std::list<std::shared_ptr<vertex_type>>::iterator         vertex_iterator;
      typedef typename std::list<std::shared_ptr<halfedge_type>>::iterator       halfedge_iterator;
      typedef typename std::list<std::shared_ptr<edge_type>>::iterator           edge_iterator;
      typedef typename std::list<std::shared_ptr<face_type>>::iterator           face_iterator;
      typedef typename std::list<std::shared_ptr<vertex_type>>::const_iterator   const_vertex_iterator;
      typedef typename std::list<std::shared_ptr<halfedge_type>>::const_iterator const_halfedge_iterator;
      typedef typename std::list<std::shared_ptr<edge_type>>::const_iterator     const_edge_iterator;
      typedef typename std::list<std::shared_ptr<face_type>>::const_iterator     const_face_iterator;

    private:

      /**
      * Default Constructed Handle:
      *
      * Null handle, Equivalent to a null-pointer or out-of-bound state.
      *
      */
      class Handle
      {
      protected:

        index_type m_idx;

      public:

        Handle()                        : m_idx(~0u)     {}
        explicit Handle(index_type idx) : m_idx(idx)     {}
        Handle(Handle const & h)        : m_idx(h.m_idx) {}

        Handle &   operator= (Handle const & h)       { m_idx = h.m_idx; return *this; }
        bool       operator< (Handle const & h) const { return m_idx < h.m_idx; }
        bool       operator==(Handle const & h) const { return (h.m_idx==m_idx); }
        bool       operator!=(Handle const & h) const { return (h.m_idx!=m_idx); }
        index_type get_idx() const { return m_idx; }
        bool       is_null() const { return (m_idx == ~0u); }
      };

    public:

      class vertex_handle : public Handle
      {
      public:
        vertex_handle()                        : Handle()    {}
        vertex_handle(index_type idx)          : Handle(idx) {}
        vertex_handle(vertex_handle const & v) : Handle(v)   {}
      };

      class halfedge_handle : public Handle
      {
      public:
        halfedge_handle()                          : Handle()    {}
        halfedge_handle(index_type idx)            : Handle(idx) {}
        halfedge_handle(halfedge_handle const & h) : Handle(h)   {}
      };

      class edge_handle : public Handle
      {
      public:
        edge_handle()                      : Handle()    {}
        edge_handle(index_type idx)        : Handle(idx) {}
        edge_handle(edge_handle const & e) : Handle(e)   {}
      };

      class face_handle : public Handle
      {
      public:
        face_handle()                      : Handle()    {}
        face_handle(index_type idx)        : Handle(idx) {}
        face_handle(face_handle const & f) : Handle(f)   {}
      };

    public:

      static vertex_handle const & null_vertex_handle()
      {
        static vertex_handle h;
        return h;
      }
      static halfedge_handle const & null_halfedge_handle()
      {
        static halfedge_handle h;
        return h;
      }
      static edge_handle const & null_edge_handle()
      {
        static edge_handle h;
        return h;
      }
      static face_handle const & null_face_handle()
      {
        static face_handle h;
        return h;
      }

    private:

      std::list<std::shared_ptr<vertex_type>>   m_vertices;
      std::list<std::shared_ptr<halfedge_type>> m_halfedges;
      std::list<std::shared_ptr<edge_type>>     m_edges;
      std::list<std::shared_ptr<face_type>>     m_faces;

      // This is our iterator lookup-tables, that can contain an empty optional, corresponding to
      // a "null"-iterator
      std::unordered_map<index_type, vertex_iterator>   m_vertex_lut;
      std::unordered_map<index_type, halfedge_iterator> m_halfedge_lut;
      std::unordered_map<index_type, edge_iterator>     m_edge_lut;
      std::unordered_map<index_type, face_iterator>     m_face_lut;

    public:

      vertex_iterator   vertex_begin()   { return m_vertices.begin(); }
      vertex_iterator   vertex_end()     { return m_vertices.end(); }
      halfedge_iterator halfedge_begin() { return m_halfedges.begin(); }
      halfedge_iterator halfedge_end()   { return m_halfedges.end(); }
      edge_iterator     edge_begin()     { return m_edges.begin(); }
      edge_iterator     edge_end()       { return m_edges.end(); }
      face_iterator     face_begin()     { return m_faces.begin(); }
      face_iterator     face_end()       { return m_faces.end(); }

      const_vertex_iterator   vertex_begin()   const { return m_vertices.begin(); }
      const_vertex_iterator   vertex_end()     const { return m_vertices.end(); }
      const_halfedge_iterator halfedge_begin() const { return m_halfedges.begin(); }
      const_halfedge_iterator halfedge_end()   const { return m_halfedges.end(); }
      const_edge_iterator     edge_begin()     const { return m_edges.begin(); }
      const_edge_iterator     edge_end()       const { return m_edges.end(); }
      const_face_iterator     face_begin()     const { return m_faces.begin(); }
      const_face_iterator     face_end()       const { return m_faces.end(); }

      size_type size_faces()     const { return m_faces.size(); }
      size_type size_halfedges() const { return m_halfedges.size(); }
      size_type size_edges()     const { return m_edges.size(); }
      size_type size_vertices()  const { return m_vertices.size(); }

      std::list<std::shared_ptr<halfedge_type>>       halfedges()       { return m_halfedges; }
      std::list<std::shared_ptr<halfedge_type>> const halfedges() const { return m_halfedges; }
      std::list<std::shared_ptr<vertex_type>>         vertices()        { return m_vertices; }
      std::list<std::shared_ptr<vertex_type>>   const vertices()  const { return m_vertices; }
      std::list<std::shared_ptr<edge_type>>           edges()           { return m_edges; }
      std::list<std::shared_ptr<edge_type>>     const edges()     const { return m_edges; }
      std::list<std::shared_ptr<face_type>>           faces()           { return m_faces; }
      std::list<std::shared_ptr<face_type>>     const faces()     const { return m_faces; }

    public:

      PolyMeshListKernel() {}

      explicit PolyMeshListKernel(PolyMeshListKernel const & other_kernel){ *this = other_kernel; }

    public:

      PolyMeshListKernel & operator=(PolyMeshListKernel const & rhs)
      {
        clear();

        //--- Brute force copy of lists
        std::copy( rhs.m_vertices.begin(),  rhs.m_vertices.end(),  std::back_inserter(m_vertices) );
        std::copy( rhs.m_halfedges.begin(), rhs.m_halfedges.end(), std::back_inserter(m_halfedges) );
        std::copy( rhs.m_edges.begin(),     rhs.m_edges.end(),     std::back_inserter(m_edges) );
        std::copy( rhs.m_faces.begin(),     rhs.m_faces.end(),     std::back_inserter(m_faces) );
        //--- Allocate storage for lookup tables (lut's)
        m_vertex_lut.resize  ( rhs.m_vertex_lut.size() );
        m_halfedge_lut.resize( rhs.m_halfedge_lut.size() );
        m_edge_lut.resize    ( rhs.m_edge_lut.size() );
        m_face_lut.resize    ( rhs.m_face_lut.size() );
        //--- Iterate lists, get self handles and update entries in lut's
        for(vertex_iterator v = vertex_begin(); v!=vertex_end(); ++v)
        {
          assert(v->get_handle().get_idx()<m_vertex_lut.size());
          m_vertex_lut[v->get_handle().get_idx()] = v;
        }
        for(halfedge_iterator h = halfedge_begin(); h!=halfedge_end(); ++h)
        {
          assert(h->get_handle().get_idx()<m_halfedge_lut.size());
          m_halfedge_lut[h->get_handle().get_idx()] = h;
        }
        for(edge_iterator e = edge_begin(); e!=edge_end(); ++e)
        {
          assert(e->get_handle().get_idx()<m_edge_lut.size());
          m_edge_lut[e->get_handle().get_idx()] = e;
        }
        for(face_iterator f = face_begin(); f!=face_end(); ++f)
        {
          assert(f->get_handle().get_idx()<m_face_lut.size());
          m_face_lut[f->get_handle().get_idx()] = f;
        }
        return (*this);
      }

    protected:

      vertex_handle create_vertex()
      {
        // Create vertex
        auto v = std::make_shared<vertex_type>();
        auto vertex_index = m_vertices.size();
        m_vertices.push_back(v);

        // Get iterator
        auto last = std::next(m_vertices.begin(), vertex_index);
        auto lut_index = m_vertex_lut.size();
        m_vertex_lut.emplace(lut_index, last);

        // Set handle
        vertex_handle h(lut_index);
        polymesh_core_access::set_self_handle(v,h);
        return h;
      }

      halfedge_handle create_halfedge()
      {
        // Create edge
        auto e = std::make_shared<halfedge_type>();
        auto edge_index = m_halfedges.size();
        m_halfedges.push_back(e);

        // Get iterator
        auto last = std::next(m_halfedges.begin(), edge_index);
        auto lut_index = m_halfedge_lut.size();
        m_halfedge_lut.emplace(lut_index, last);

        // Set handle
        halfedge_handle h(lut_index);
        polymesh_core_access::set_self_handle(e,h);
        return h;
      }

      edge_handle create_edge()
      {
        // Create edge
        auto e = std::make_shared<edge_type>();
        auto edge_index = m_edges.size();
        m_edges.push_back(e);

        // Get iterator
        auto last = std::next(m_halfedges.begin(), edge_index);
        auto lut_index = m_edge_lut.size();
        m_edge_lut.emplace(lut_index, last);

        // Set handle
        edge_handle h(lut_index);
        polymesh_core_access::set_self_handle(e,h);
        return h;
      }

      face_handle create_face()
      {
        // Create face
        auto f = std::make_shared<face_type>();
        auto face_index = m_faces.size();
        m_faces.push_back(f);

        // Get iterator
        auto last = std::next(m_faces.begin(), face_index);
        auto lut_index = m_face_lut.size();
        m_face_lut.emplace(lut_index, last);

        // Set handle
        face_handle h(lut_index);
        polymesh_core_access::set_self_handle(f,h);
        return h;
      }

      void erase_vertex(vertex_handle const & v)
      {
        auto vertex_index = v.get_idx();
        assert(vertex_index >= 0);

        auto vit = m_vertex_lut.find(vertex_index);
        if(vit != m_vertex_lut.end())
        {
          m_vertices.erase(vit);
          m_vertex_lut.erase(vertex_index);
        }
      }

      void erase_halfedge(halfedge_handle const & h)
      {
        auto edge_index = h.get_idx();
        assert(edge_index>=0);

        auto hit = m_halfedge_lut.find(edge_index);
        if(hit != m_halfedge_lut.end())
        {
          m_halfedges.erase(hit);
          m_halfedge_lut.erase(edge_index);
        }
      }

      void erase_edge(edge_handle const & e)
      {
        auto edge_index = e.get_idx();
        assert(e.get_idx()>=0);

        auto eit = m_edge_lut.find(e.get_idx());
        if(eit != m_edge_lut.end())
        {
          m_edges.erase(eit);
          m_edge_luterase(e.get_idx());
        }
      }

      void erase_face(face_handle const & f)
      {
        auto face_index = f.get_idx();
        assert(f.get_idx()>=0);

        auto fit = m_face_lut.find(f.get_idx());
        if(fit != m_face_lut.end())
        {
          m_faces.erase(fit);
          m_face_lut.erase(f.get_idx());
        }
      }

    public:

      vertex_handle get_vertex_handle(index_type idx) const
      {
        assert(idx>=0);

        bool exists = m_vertex_lut.find(idx) != m_vertex_lut.end();
        std::shared_ptr<vertex_type> vertex_ptr = exists ? *m_vertex_lut[idx] : nullptr;
        return vertex_ptr ? vertex_ptr->get_handle() : this->null_vertex_handle();
      }

      halfedge_handle get_halfedge_handle(index_type idx) const
      {
        assert(idx>=0);

        bool exists = m_halfedge_lut.find(idx) != m_halfedge_lut.end();
        std::shared_ptr<halfedge_type> edge_ptr = exists ? *m_halfedge_lut[idx] : nullptr;
        return edge_ptr ? edge_ptr->get_handle() : this->null_halfedge_handle();
      }

      edge_handle get_edge_handle(index_type idx) const
      {
        assert(idx>=0);

        bool exists = m_edge_lut.find(idx) != m_edge_lut.end();
        std::shared_ptr<edge_type> edge_ptr = exists ? *m_edge_lut[idx] : nullptr;
        return edge_ptr ? edge_ptr->get_handle() : this->null_halfedge_handle();
      }

      face_handle get_face_handle(index_type idx) const
      {
        assert(idx>=0);

        bool exists = m_face_lut.find(idx) != m_face_lut.end();
        std::shared_ptr<face_type> face_ptr = exists ? *m_face_lut[idx] : nullptr;
        return face_ptr ? face_ptr->get_handle() : this->null_halfedge_handle();
      }

      vertex_iterator get_vertex_iterator(vertex_handle const & v) /*const*/
      {
        if( v == this->null_vertex_handle() )
          return vertex_end();

        assert(v.get_idx()>=0);

        if( m_vertex_lut.find(v.get_idx()) == m_vertex_lut.end())
          return vertex_end();

        return m_vertex_lut[v.get_idx()];
      }

      halfedge_iterator get_halfedge_iterator(halfedge_handle const & h) /*const*/
      {
        if( h == this->null_halfedge_handle() )
          return halfedge_end();

        assert(h.get_idx()>=0);

        if( m_halfedge_lut.find(h.get_idx()) == m_halfedge_lut.end())
          return halfedge_end();

        return  m_halfedge_lut[h.get_idx()];
      }

      edge_iterator get_edge_iterator(edge_handle const & e) /*const*/
      {
        if( e == this->null_edge_handle() )
          return edge_end();

        assert(e.get_idx()>=0);

        if( m_edge_lut.find(e.get_idx()) == m_edge_lut.end())
          return edge_end();

        return  m_edge_lut[e.get_idx()];
      }

      face_iterator get_face_iterator(face_handle const & f) /*const*/
      {
        if( f == this->null_face_handle() )
          return face_end();

        assert(f.get_idx()>=0);

        if( m_face_lut.find(f.get_idx()) == m_face_lut.end())
          return face_end();

        return  m_face_lut[f.get_idx()];
      }

      void clear()
      {
        m_vertices.clear();
        m_halfedges.clear();
        m_edges.clear();
        m_faces.clear();
        m_vertex_lut.clear();
        m_halfedge_lut.clear();
        m_edge_lut.clear();
        m_face_lut.clear();
      }

    public:

      bool is_valid_vertex_handle(vertex_handle const & v) const
      {
        if(v == this->null_vertex_handle())
          return false;

        assert(v.get_idx()>=0);

        return m_vertex_lut.find(v.get_idx()) != m_vertex_lut.end();
      }

      bool is_valid_halfedge_handle(halfedge_handle const & h) const
      {
        if(h == this->null_halfedge_handle())
          return false;

        assert(h.get_idx()>=0);

        return m_halfedge_lut.find(h.get_idx()) != m_halfedge_lut.end();
      }

      bool is_valid_edge_handle(edge_handle const & e) const
      {
        if(e == this->null_edge_handle())
          return false;

        assert(e.get_idx()>=0);

        return m_edge_lut.find(e.get_idx()) != m_edge_lut.end();
      }

      bool is_valid_face_handle(face_handle const & f) const
      {
        if(f == this->null_face_handle())
          return false;

        assert(f.get_idx()>=0);

        return m_face_lut.find(f.get_idx()) != m_face_lut.end();
      }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_KERNELS_POLYMESH_LIST_KERNEL2_H
#endif
