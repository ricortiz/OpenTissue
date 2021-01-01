#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_CORE_ACCESS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_CORE_ACCESS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>
#include <memory>

namespace OpenTissue
{
  namespace polymesh
  {
    class polymesh_core_access
    {
    public:

      template<typename feature_type,typename handle>
      static void set_self_handle(std::shared_ptr<feature_type> feature,handle self)
      {
        feature->set_handle(self);
      }

      template<typename feature_type,typename mesh_type>
      static void set_owner(std::shared_ptr<feature_type> feature, std::shared_ptr<mesh_type> owner)
      {
        feature->set_owner(owner);
      }

      template<typename vertex_type,typename halfedge_handle>
      static void set_outgoing_halfedge_handle(std::shared_ptr<vertex_type> v,halfedge_handle h)
      {
        v->set_outgoing_halfedge_handle(h);
      }

      template<typename halfedge_type,typename halfedge_handle>
      static void set_next_handle(std::shared_ptr<halfedge_type> h,halfedge_handle n)
      {
        h->set_next_handle(n);
      }

      template<typename halfedge_type,typename face_handle>
      static void set_face_handle(std::shared_ptr<halfedge_type> h,face_handle f)
      {
        h->set_face_handle(f);
      }

      template<typename halfedge_type,typename halfedge_handle>
      static void set_twin_handle(std::shared_ptr<halfedge_type> h,halfedge_handle t)
      {
        h->set_twin_handle(t);
      }

      template<typename halfedge_type,typename vertex_handle>
      static void set_destination_handle(std::shared_ptr<halfedge_type> h,vertex_handle v)
      {
        h->set_destination_handle(v);
      }

      template<typename face_type,typename halfedge_handle>
      static void set_border_halfedge_handle(std::shared_ptr<face_type> f,halfedge_handle h)
      {
        f->set_border_halfedge_handle(h);
      }

      template<typename halfedge_type,typename edge_handle>
      static void set_edge_handle(std::shared_ptr<halfedge_type> h,edge_handle e){ h->set_edge_handle(e); }

      template<typename edge_type,typename halfedge_handle>
      static void set_halfedge0_handle(std::shared_ptr<edge_type> e,halfedge_handle h){ e->set_halfedge0_handle(h); }

      template<typename edge_type,typename halfedge_handle>
      static void set_halfedge1_handle(std::shared_ptr<edge_type> e,halfedge_handle h){ e->set_halfedge1_handle(h); }



      /**
      * This method makes sure that the outgoing halfedge from a vertex
      * is a boundary half-edge (i.e. it has no incident face) if one exist.
      */
      template<typename vertex_type>
      static void adjust_outgoing_halfedge_handle(std::shared_ptr<vertex_type> v)
      {
        typedef typename vertex_type::mesh_type                  mesh_type;
        typedef typename mesh_type::vertex_halfedge_circulator   vertex_halfedge_circulator;

        for(auto circulator : vertex_halfedge_circulator(v))
        {
          if(circulator->get_face_handle().is_null())
          {
            set_outgoing_halfedge_handle(v,circulator->get_handle());
            return;
          }
        }
      }

      /**
      * Unlinks a halfedge (and its twin) from its destination vertex.
      *
      * @param h   The halfedge to be unlinked.
      *
      * @return    If the halfedge was unlinked then the return value is true, otherwise it is false.
      */
      template<typename halfedge_type>
      bool unlink(std::shared_ptr<halfedge_type> h)
      {
        typedef typename halfedge_type::mesh_type       mesh_type;
        //typedef typename mesh_type::halfedge_iterator   halfedge_iterator;
        typedef typename mesh_type::halfedge_iterator   vertex_iterator;
        //                                    //
        //  \     h1_prev                     //
        //  _\|                               //
        //    \                               //
        //     \                              //
        //      \          h1                 //
        //        v   -------->----------     //
        //        *                           //
        //       /    --------<----------     //
        //      /          h0 (h)             //
        //    |/_                             //
        //    /                               //
        //        h0_next                     //
        //                                    //
        if(h->get_destination_handle().is_null())
        {
          assert(!"unlink(h): Illegal topology!");
          return false;
        }

        // Get a pointer to owner
        auto owner = h.get_owner();
        if(!owner)
        {
          assert(!"unlink(h): No owner mesh!");
          return false;
        }

        //--- Get pointers for all
        auto h1      = *h->get_twin_iterator();
        auto h0      = *h1->get_twin_itreator();
        auto v       = *h.get_destination_iterator();
        auto h1_prev = *h1->get_prev_iterator();
        auto h0_next = *h0->get_next_iterator();

        //--- more than one incident edge to vertex
        if(h1_prev->m_self != h0->m_self)
        {
          h1_prev->m_next   = h0_next->m_self;
          h0_next->m_prev   = h1_prev->m_self;
        }

        //--- exactly one incident edge (the one we are aboud to unlink) to vertex
        if(h1_prev->m_self == h0->m_self)
        {
          v->m_outgoing_halfedge = owner->null_halfedge_handle();
        }

        //--- more than one incident edge to vertex, but we are removing the edge v is pointing to.
        if(h1->m_self == v->m_outgoing_halfedge)
        {
          v->m_outgoing_halfedge = h0_next->m_self;
        }

        h0->m_next        = owner->null_halfedge_handle();
        h0->m_destination = owner->null_vertex_handle();
        h1->m_prev        = owner->null_halfedge_handle();
        adjust_outgoing_halfedge_handle(v);
        return true;
      }

      /**
      * Links halfedge into v's topology, such that h points to v.
      *
      * @param h0   The halfedge that should have v as its new destination.
      * @param v    The vertex, must have an empty gap.
      *
      * @return     If succesfully linked the return value is true otherwise it is false.
      */
      template<typename halfedge_type,typename vertex_type>
      bool link(std::shared_ptr<halfedge_type> h0,std::shared_ptr<vertex_type> v)
      {
        //typedef typename halfedge_iterator::value_type  halfedge_type;
        //typedef typename halfedge_type::mesh_type       mesh_type:
        //typedef typename mesh_type::halfedge_iterator halfedge_iterator;
        //typedef typename mesh_type::halfedge_iterator vertex_iterator;
        //                                    //
        //  \     h1_prev                     //
        //  _\|                               //
        //    \                               //
        //     \                              //
        //      \          h1                 //
        //        v   -------->----------     //
        //        *                           //
        //       /    --------<----------     //
        //      /          h0 (h)             //
        //    |/_                             //
        //    /                               //
        //        h0_next                     //
        //                                    //
        if(!h0->get_destination_handle().is_null())
        {
          assert(!"link(h,v): Illegal topology, h allready had a destination!");
          return false;
        };

        auto h1 = *h0->get_twin_iterator();

        //--- vertex is isolated
        if(v->m_outgoing_halfedge.is_null())
        {
          v->m_outgoing_halfedge = h1->m_self;
          h0->m_destination = v->m_self;
          h0->m_next = h1->m_self;
          h1->m_prev = h0->m_self;
          return true;
        }

        //--- Test for empty gap in vertex one-ring neighborhood
        auto h0_next = *v->get_outgoing_halfedge_iterator();
        if(!h0_next->get_face_handle().is_null())
        {
          assert(!"link(h,v): vertex v did not have an empty gap");
          return false;
        }
        auto h1_prev = *h0_next->get_prev_iterator();
        if(!h1_prev->get_face_handle().is_null())
        {
          assert(!"link(h,v): vertex v did not have an empty gap");
          return false;
        }

        //--- Now link everything together
        h0->m_next        = h0_next->m_self;
        h0_next->m_prev   = h0->m_self;
        h1_prev->m_next   = h1->m_self;
        h1->m_prev        = h1_prev->m_self;
        h0->m_destination =  v->m_self;

        adjust_outgong_edge(v);

        return true;
      }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_CORE_ACCESS_H
#endif
