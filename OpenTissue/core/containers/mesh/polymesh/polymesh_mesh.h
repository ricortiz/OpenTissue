#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_MESH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_halfedge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_core_access.h>

#include <OpenTissue/core/containers/mesh/polymesh/circulators/polymesh_circulators.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_boundary.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>

#include <vector>
#include <memory>
#include <algorithm>

namespace OpenTissue
{
  namespace polymesh
  {
    namespace detail
    {

      /**
      * The first template argument is supposed to be a math types type binder.
      * OpenTissue provides a simple basic math type-binder in the math sub-library.
      * See OpenTissue::math::BasicMathTypes<real_type, size_type>
      *
      * The next four template arguments are supposed to be vertex traits,
      * halfedge traits, edge traits, and face traits. The last template
      * argument is the polymesh kernel type that is supposed to be used.
      */
      template<
          typename MT
        , typename V
        , typename H
        , typename E
        , typename F
        , typename M
        , template <typename, typename, typename,typename> class K
      >
      class PMesh : public M, public K<
          PolyMeshVertex<   PMesh<MT,V,H,E,F,M,K> >
        , PolyMeshHalfEdge< PMesh<MT,V,H,E,F,M,K> >
        , PolyMeshEdge<     PMesh<MT,V,H,E,F,M,K> >
        , PolyMeshFace<     PMesh<MT,V,H,E,F,M,K> >
        >
        , public std::enable_shared_from_this<PMesh<MT,V,H,E,F,M,K>
      >
      {
      public:

        typedef MT                                                    math_types;
        typedef V                                                     vertex_traits;
        typedef H                                                     halfedge_traits;
        typedef E                                                     edge_traits;
        typedef F                                                     face_traits;
        typedef M                                                     mesh_traits;
        typedef PMesh<MT,V,H,E,F,M,K>                                 mesh_type;
        typedef PolyMeshVertex<mesh_type>                             vertex_type;
        typedef PolyMeshHalfEdge<mesh_type>                           halfedge_type;
        typedef PolyMeshEdge<mesh_type>                               edge_type;
        typedef PolyMeshFace<mesh_type>                               face_type;

        typedef K<vertex_type, halfedge_type, edge_type, face_type  > kernel_type;

        typedef typename kernel_type::vertex_handle           vertex_handle;
        typedef typename kernel_type::halfedge_handle         halfedge_handle;
        typedef typename kernel_type::edge_handle             edge_handle;
        typedef typename kernel_type::face_handle             face_handle;

        typedef typename kernel_type::vertex_iterator         vertex_iterator;
        typedef typename kernel_type::halfedge_iterator       halfedge_iterator;
        typedef typename kernel_type::edge_iterator           edge_iterator;
        typedef typename kernel_type::face_iterator           face_iterator;

        typedef typename kernel_type::const_vertex_iterator   const_vertex_iterator;
        typedef typename kernel_type::const_halfedge_iterator const_halfedge_iterator;
        typedef typename kernel_type::const_edge_iterator     const_edge_iterator;
        typedef typename kernel_type::const_face_iterator     const_face_iterator;

      public:

        typedef PolyMeshVertexVertexCirculator<PMesh,vertex_type>           vertex_vertex_circulator;
        typedef PolyMeshVertexHalfedgeCirculator<PMesh,halfedge_type>       vertex_halfedge_circulator;
        typedef PolyMeshVertexEdgeCirculator<PMesh,edge_type>               vertex_edge_circulator;
        typedef PolyMeshVertexFaceCirculator<PMesh,face_type>               vertex_face_circulator;

        typedef PolyMeshVertexVertexCirculator<PMesh,vertex_type const>     const_vertex_vertex_circulator;
        typedef PolyMeshVertexHalfedgeCirculator<PMesh,halfedge_type const> const_vertex_halfedge_circulator;
        typedef PolyMeshVertexEdgeCirculator<PMesh,edge_type const>         const_vertex_edge_circulator;
        typedef PolyMeshVertexFaceCirculator<PMesh,face_type const>         const_vertex_face_circulator;

        typedef PolyMeshFaceVertexCirculator<PMesh,vertex_type>             face_vertex_circulator;
        typedef PolyMeshFaceHalfedgeCirculator<PMesh,halfedge_type>         face_halfedge_circulator;
        typedef PolyMeshFaceEdgeCirculator<PMesh,edge_type>                 face_edge_circulator;
        typedef PolyMeshFaceFaceCirculator<PMesh,face_type>                 face_face_circulator;

        typedef PolyMeshFaceVertexCirculator<PMesh,vertex_type const>       const_face_vertex_circulator;
        typedef PolyMeshFaceHalfedgeCirculator<PMesh,halfedge_type const>   const_face_halfedge_circulator;
        typedef PolyMeshFaceEdgeCirculator<PMesh,edge_type const>           const_face_edge_circulator;
        typedef PolyMeshFaceFaceCirculator<PMesh,face_type const>           const_face_face_circulator;

      private:

        struct assign_owner
        {
          assign_owner(std::shared_ptr<mesh_type> new_owner)
            : m_new_owner(new_owner)
          {}

          template <typename feature_type>
          void operator() (std::shared_ptr<feature_type> f)
          {
            polymesh_core_access::set_owner(f, m_new_owner);
          }

          std::shared_ptr<mesh_type> m_new_owner;
        };

      public:

        PMesh(){}

        ~PMesh()        {          this->clear();        }

        explicit PMesh(PMesh const & m)  { (*this) = m; }

        PMesh & operator=(PMesh const & mesh)
        {
          if ( this == &mesh ) return *this; // Do nothing on assignment to self.

          kernel_type::operator=(mesh);

          //--- Reassign owner pointers of copied data
          std::for_each( this->vertex_begin(),   this->vertex_end(),   assign_owner(this) );
          std::for_each( this->halfedge_begin(), this->halfedge_end(), assign_owner(this) );
          std::for_each( this->edge_begin(),     this->edge_end(),     assign_owner(this) );
          std::for_each( this->face_begin(),     this->face_end(),     assign_owner(this) );

          return *this;
        }

      public:

        halfedge_iterator find_halfedge_iterator(std::shared_ptr<vertex_type> vertex_a, std::shared_ptr<vertex_type> vertex_b)
        {
          for(auto circulator : vertex_halfedge_circulator(vertex_a))
          {
            if(circulator->get_destination_handle()==vertex_b->get_handle())
              return this->get_halfedge_iterator(circulator->get_handle());
          }
          return this->halfedge_end();
        }

        halfedge_handle find_halfedge_handle(vertex_handle handle_a, vertex_handle handle_b)
        {
          if(! kernel_type::is_valid_vertex_handle(handle_a))
            return this->null_halfedge_handle();
          if(! kernel_type::is_valid_vertex_handle(handle_b))
            return this->null_halfedge_handle();

          auto vertex = *kernel_type::get_vertex_iterator(handle_a);
          for(auto circulator : vertex_halfedge_circulator(vertex))
          {
            if(circulator->get_destination_handle()==handle_b)
              return circulator->get_handle();
          }
          return this->null_halfedge_handle();
        }

        edge_handle find_edge_handle(vertex_handle handle_a, vertex_handle handle_b)
        {
          halfedge_handle h = this->find_halfedge_handle(handle_a,handle_b);
          if(h.is_null())
            return this->null_edge_handle();
          return kernel_type::get_halfedge_iterator(h)->get_edge_handle();
        }

      private:

        /**
        * Creates a halfedge from vertex_a to vertex_b if one does not already exist. If
        * one exist then it is returned instead.
        *
        * In case a new halfedge is created, then vertex destination, and
        * twin pointers are set. However, face and next pointers are set to null.
        *
        */
        std::shared_ptr<halfedge_type> add_halfedge(std::shared_ptr<vertex_type> vertex_a, std::shared_ptr<vertex_type> vertex_b)
        {
          auto lookup = this->find_halfedge_iterator(vertex_a,vertex_b);
          if(lookup != this->halfedge_end() )
            return *lookup;

          edge_handle whole_edge_handle     = this->create_edge();
          halfedge_handle edge_handle0 = this->create_halfedge();
          halfedge_handle edge_handle1 = this->create_halfedge();

          auto h = *this->get_halfedge_iterator(edge_handle0);
          auto t = *this->get_halfedge_iterator(edge_handle1);
          auto e = *this->get_edge_iterator(whole_edge_handle);

          polymesh_core_access::set_owner(h,this->shared_from_this());
          polymesh_core_access::set_next_handle(h,this->null_halfedge_handle());
          polymesh_core_access::set_face_handle(h,this->null_face_handle());
          polymesh_core_access::set_twin_handle(h,edge_handle1);
          polymesh_core_access::set_destination_handle(h,vertex_b->get_handle());
          polymesh_core_access::set_edge_handle(h,whole_edge_handle);

          polymesh_core_access::set_owner(t,this->shared_from_this());
          polymesh_core_access::set_next_handle(t,this->null_halfedge_handle());
          polymesh_core_access::set_face_handle(t,this->null_face_handle());
          polymesh_core_access::set_twin_handle(t,edge_handle0);
          polymesh_core_access::set_destination_handle(t,vertex_a->get_handle());
          polymesh_core_access::set_edge_handle(t,whole_edge_handle);

          polymesh_core_access::set_owner(e,this->shared_from_this());
          polymesh_core_access::set_halfedge0_handle(e,edge_handle0);
          polymesh_core_access::set_halfedge1_handle(e,edge_handle1);

          return h;
        }

      public:

        vertex_handle add_vertex()
        {
          vertex_handle v_handle = this->create_vertex();
          auto v = *kernel_type::get_vertex_iterator(v_handle);
          polymesh_core_access::set_owner(v,this->shared_from_this());
          return v_handle;
        }

        template<typename vector3_type>
        vertex_handle add_vertex(vector3_type const & coord)
        {
          vertex_handle v_handle = add_vertex();
          (*kernel_type::get_vertex_iterator(v_handle))->m_coord = coord;
          return v_handle;
        }

        template<typename vertex_handle_iterator>
        face_handle add_face(vertex_handle_iterator begin,vertex_handle_iterator end)
        {
          if(begin==end)
          {
            assert(!"PMesh::insert_face(...): Could not create face, first and last vertex were the same.");
            return this->null_face_handle();
          }

          std::size_t n = std::distance(begin,end);
          if(n<3)
          {
            assert(!"PMesh::insert_face(...): Could not create face, not enough vertices.");
            return this->null_face_handle();
          }

          //--- Test all vertex handles are valid.
          //--- Test that all vertices have a gap in their one-ring neighborhood i.e a at least one wedge without a face pointer.
          std::vector<std::shared_ptr<vertex_type>> Vs;

          for(auto vhit = begin; vhit != end; ++vhit)
          {
            vertex_handle v = *vhit;
            if(!this->is_valid_vertex_handle(v))
            {
              assert(!"PMesh::insert_face(...): Could not create face, invalid vertex handle encountered.");
              return this->null_face_handle();
            }

            auto vertex_pointer = *this->get_vertex_iterator( v );
            if(!is_boundary(vertex_pointer))
            {
              assert(!"PMesh::insert_face(...): Could not create face, 2 manifold vertex encountered.");
              return this->null_face_handle();
            }

            Vs.push_back(vertex_pointer);
          }

          //--- Test if any edges exist on boundary, if so they should not have any incident face.
          std::vector<std::shared_ptr<halfedge_type>> Es;
          std::vector<bool> is_new(n+1, false);
          for(size_t i = 0; i < n; ++i)
          {
            size_t j = (i + 1) % n;
            auto h = this->find_halfedge_handle(Vs[i]->get_handle(), Vs[j]->get_handle());
            if(h.is_null())
            {
              //std::cout << "  new edge(" << vi->get_handle().get_idx()<< "," <<  vj->get_handle().get_idx() << ")"  << std::endl;
              is_new[i] = true;
              //--- Create missing edges (they are not linked into vertex neighborhoods yet!!!
              Es.push_back(this->add_halfedge(Vs[i],Vs[j]));
            }
            else
            {
              //std::cout << "  old edge(" << vi->get_handle().get_idx()<< "," <<  vj->get_handle().get_idx() << ")"  << std::endl;
              is_new[i] = false;
              if(!this->is_valid_halfedge_handle(h))
              {
                assert(!"PMesh::insert_face(...): Could not create a valid halfedge.");
                return this->null_face_handle();
              }

              auto edge_pointer = *this->get_halfedge_iterator(h);
              if(!is_boundary(edge_pointer))
              {
                assert(!"PMesh::insert_face(...): Could not create face, encountered halfedge belonging to another face.");
                return this->null_face_handle();
              }

              Es.push_back(edge_pointer);
            }
          }

          //--- Test if edges form empty gap on vertices if not re-arrange vertex-connectity
          for(size_t i = 0; i < n; ++i)
          {
            size_t j = (i + 1) % n;
            if ( (!is_new[i]) && (!is_new[j]) )
            {
              std::shared_ptr<halfedge_type> inner_prev = Es[i];
              std::shared_ptr<halfedge_type> inner_next = Es[j];
              if (inner_prev->get_next_handle() != inner_next->get_handle())
              {
                //--- Search a free gap
                //--- Free gap will be between boundary_prev and boundary_next
                std::shared_ptr<halfedge_type> outer_prev = *inner_next->get_twin_iterator();
                std::shared_ptr<halfedge_type> boundary_prev = outer_prev;
                do
                {
                  std::shared_ptr<halfedge_type> p = *boundary_prev->get_next_iterator();
                  boundary_prev = *p->get_twin_iterator();
                }  while (!is_boundary(boundary_prev) || boundary_prev == inner_prev);

                if(!is_boundary(boundary_prev))
                {
                  assert(!"PMesh::add_face(...): Could not create face, vertex neighborhood flawed.");
                  return this->null_face_handle();
                }

                if (boundary_prev == inner_prev)
                {
                  assert(!"PMesh::add_face(...): Could not create face, vertex neighborhood flawed.");
                  return this->null_face_handle();
                }

                std::shared_ptr<halfedge_type> boundary_next = *boundary_prev->get_next_iterator();
                if(!is_boundary(boundary_next))
                {
                  assert(!"PMesh::add_face(...): Could not create face, vertex neighborhood flawed.");
                  return this->null_face_handle();
                }

                if (boundary_next == inner_next)
                {
                  assert(!"PMesh::add_face(...): Could not create face, vertex neighborhood flawed.");
                  return this->null_face_handle();
                }

                //--- Get stuff that is in the way
                std::shared_ptr<halfedge_type> patch_start = *inner_prev->get_next_iterator();
                std::shared_ptr<halfedge_type> patch_end  = *inner_next->get_prev_iterator();

                //--- Move stuff to somewhere else
                polymesh_core_access::set_next_handle( boundary_prev, patch_start->get_handle()  );
                polymesh_core_access::set_next_handle( patch_end,     boundary_next->get_handle());
                polymesh_core_access::set_next_handle( inner_prev,    inner_next->get_handle()   );

                //std::cout << "-- created empty gap at edge("
                //  << Vs[i]->get_handle().get_idx()
                //  << ","
                //  << Vs[j]->get_handle().get_idx()
                //  << ") and edge("
                //  << Vs[j]->get_handle().get_idx()
                //  << ","
                //  << Vs[(j+1)%n]->get_handle().get_idx()
                //  << ")"
                //  << std::endl;
              }
              //else
              //{
              //  std::cout << "-- exist empty gap at edge("
              //    << Vs[i]->get_handle().get_idx()
              //    << ","
              //    << Vs[j]->get_handle().get_idx()
              //    << ") and edge("
              //    << Vs[j]->get_handle().get_idx()
              //    << ","
              //    << Vs[(j+1)%n]->get_handle().get_idx()
              //    << ")"
              //    << std::endl;
              //}
            }
          }

          //--- Create the face
          face_handle f = this->create_face();
          auto fptr = *this->get_face_iterator(f);
          polymesh_core_access::set_owner(fptr, this->shared_from_this());
          polymesh_core_access::set_border_halfedge_handle(fptr, Es[n-1]->get_handle());

          //--- Setup halfedges
          std::vector<bool>  needs_adjust(n, false);

          for (size_t i=0, j=1; i<n; ++i, ++j, j%=n)
          {
            auto v = Vs[j];
            auto inner_prev = Es[i];
            auto inner_next = Es[j];

            int id = 0;
            if (is_new[i])
              id |= 1;
            if (is_new[j])
              id |= 2;
            if (id)
            {
              auto outer_prev = *inner_next->get_twin_iterator();
              auto outer_next = *inner_prev->get_twin_iterator();
              //--- set outer links
              switch (id)
              {
              case 1: //--- inner_prev is new, inner_next is old
                {
                  //--- Notice that next pointer of outer_next will be handled by case 2?
                  auto boundary_prev = *inner_next->get_prev_iterator();
                  polymesh_core_access::set_next_handle(boundary_prev, outer_next->get_handle());
                  polymesh_core_access::set_outgoing_halfedge_handle(v, outer_next->get_handle());
                }
                break;
              case 2: //--- inner_next is new, inner_prev is old
                {
                  //--- Notice that next pointer of inner_next will be handled by case 1?
                  auto boundary_next = *inner_prev->get_next_iterator();
                  polymesh_core_access::set_next_handle(outer_prev, boundary_next->get_handle());
                  polymesh_core_access::set_outgoing_halfedge_handle(v, boundary_next->get_handle());
                }
                break;
              case 3: //---- inner_next is new, inner_prev is new
                {
                  //--- Test if v is an isolated vertex (i.e. has no outgoing halfedge)
                  if (v->get_outgoing_halfedge_handle().is_null())
                  {
                    polymesh_core_access::set_outgoing_halfedge_handle(v, outer_next->get_handle());
                    polymesh_core_access::set_next_handle(outer_prev, outer_next->get_handle());
                  }
                  else
                  {
                    //--- v is not an isolated vertex, we must link new face into existing neighborhood
                    auto boundary_next = *v->get_outgoing_halfedge_iterator();
                    if(!boundary_next->get_face_handle().is_null())
                    {
                      assert(!"PMesh::add_face(...): Outgoing halfedge from vertex was not pointing to empty gap");
                      return this->null_face_handle();
                    }
                    auto boundary_prev = *boundary_next->get_prev_iterator();
                    polymesh_core_access::set_next_handle(boundary_prev, outer_next->get_handle());
                    polymesh_core_access::set_next_handle(outer_prev, boundary_next->get_handle());
                  }
                }
                break;
              }

              polymesh_core_access::set_next_handle( inner_prev, inner_next->get_handle());
            }
            else
            {
              needs_adjust[j] = (v->get_outgoing_halfedge_handle() == inner_next->get_handle());
            }

            polymesh_core_access::set_face_handle(Es[i], f);
          }
          //--- Adjust vertices' halfedge handle too point on a empty gap...
          for (size_t i=0; i<n; ++i)
          {
            if (needs_adjust[i])
              polymesh_core_access::adjust_outgoing_halfedge_handle(Vs[i]);
          }
          return f;
        }

        face_handle add_face(vertex_handle const & v0,vertex_handle const & v1,vertex_handle const & v2)
        {
          vertex_handle handles[3];
          handles[0] = v0;
          handles[1] = v1;
          handles[2] = v2;
          return add_face(handles,handles+3);
        }

        bool remove_vertex(vertex_handle const & v)
        {
          //--- Make sure that we remove valid vertex
          if(! (this->is_valid_vertex_handle(v)) )
          {
            assert(!"PMesh::remove_vertex(...): Invalid vertex handle");
            return false;
          }
          return this->remove_vertex( *this->get_vertex_iterator(v) );
        }

        bool remove_vertex(std::shared_ptr<vertex_type> v)
        {
          //--- Make sure that vertex have empty 1-ring neighborhood
          halfedge_handle h = v->get_outgoing_halfedge_handle();
          if(!h.is_null())
          {
            assert(!"PMesh::remove_vertex(...): Could not remove vertex because it is bound to an edge");
            return false;
          }
          //--- Finally ask kernel to remove vertex
          this->erase_vertex(v->get_handle());
          return true;
        }

        bool remove_edge(edge_handle const & e)
        {
          //--- Make sure that we remove a valid edge
          if(!this->is_valid_edge_handle(e))
          {
            assert(!"PMesh::remove_edge(...): Invalid edge handle");
            return false;
          }
          return this->remove_edge( *this->get_edge_iterator(e) );
        }

        bool remove_edge(std::shared_ptr<edge_type> e)
        {
          halfedge_handle h0_handle = e->get_halfedge0_handle();
          halfedge_handle h1_handle = e->get_halfedge1_handle();
          auto h0 = *this->get_halfedge_iterator(h0_handle);
          auto h1 = *this->get_halfedge_iterator(h1_handle);

          //--- Test that edges are not connected to any faces
          if(!h0->get_face_handle().is_null())
          {
            //assert(!"PMesh::remove_edge(...): Could not remove edge because it is bound to a face");
            return false;
          }
          if(!h1->get_face_handle().is_null())
          {
            //assert(!"PMesh::remove_edge(...): Could not remove edge because it is bound to a face");
            return false;
          }

          //--- Get handles to end-vertices
          vertex_handle dest_handle0 = h0->get_destination_handle();
          vertex_handle dest_handle1 = h1->get_destination_handle();

          //--- Test that end vertices are valid
          if(!this->is_valid_vertex_handle(dest_handle0))
          {
            assert(!"PMesh::remove_edge(...): Illegal edge topology, invalid vertex handle encountered");
            return false;
          }
          if(!this->is_valid_vertex_handle(dest_handle1))
          {
            assert(!"PMesh::remove_edge(...): Illegal edge topology, invalid vertex handle encountered");
            return false;
          }
          //--- Get iterators to end vertices
          auto dest0 = *this->get_vertex_iterator(dest_handle0);
          auto dest1 = *this->get_vertex_iterator(dest_handle1);

          //--- Vertex A's connectivity -------------------------------------------
          {
            //--- first test that next and prev handles are valid
            halfedge_handle h1_prev_handle = h1->get_prev_handle();
            halfedge_handle h0_next_handle = h0->get_next_handle();
            if(!this->is_valid_halfedge_handle(h1_prev_handle))
            {
              assert(!"PMesh::remove_edge(...): Illegal edge topology, mesh is in-consistent, could not remove edge");
              return false;
            }
            if(!this->is_valid_halfedge_handle(h0_next_handle))
            {
              assert(!"PMesh::remove_edge(...): Illegal edge topology, mesh is in-consistent, could not remove edge");
              return false;
            }
            //--- get iterators for the next and prev half-edges
            //halfedge_iterator h0_next_it = kernel_type::get_halfedge_iterator(h0_next_handle);
            auto h1_prev = *this->get_halfedge_iterator(h1_prev_handle);
            //--- unlink the edge that is about to be removed from A's 1-ring
            h1_prev->set_next_handle(h0_next_handle);

            //--- Make sure that the outgoing halfedge from vertex A is stil valid
            //--- First test if A becomes an isolated vertex.
            if(h1_prev_handle==h0_handle)
            {
              dest0->set_outgoing_halfedge_handle(this->null_halfedge_handle());
            }
            //--- Test if the edge we are about to delete is the outgoing edge from A
            //--- In this case we must pick another outgoing edge
            else if(h1_handle == dest0->get_outgoing_halfedge_handle())
            {
              dest0->set_outgoing_halfedge_handle(h0_next_handle);
            }
            //--- Finally adjust outgoing halfedge to point to empty gap in 1-ring neighborhood of A
            polymesh_core_access::adjust_outgoing_halfedge_handle(dest0);
          }
          //--- Vertex B's connectivity ------------------------------------------------
          {
            //--- first test that next and prev handles are valid
            halfedge_handle h1_next_handle = h1->get_next_handle();
            halfedge_handle h0_prev_handle = h0->get_prev_handle();

            if(!this->is_valid_halfedge_handle(h1_next_handle) )
            {
              assert(!"PMesh::remove_edge(...): Illegal edge topology, mesh is in-consistent, could not remove edge");
              return false;
            }
            if(!this->is_valid_halfedge_handle(h0_prev_handle))
            {
              assert(!"PMesh::remove_edge(...): Illegal edge topology, mesh is in-consistent, could not remove edge");
              return false;
            }
            //--- get iterators for the next and prev half-edges
            //halfedge_iterator h1_next_it = kernel_type::get_halfedge_iterator(h1_next_handle);
            auto h0_prev = this->get_halfedge_iterator(h0_prev_handle);
            //--- unlink the edge that is about to be removed from B's 1-ring
            polymesh_core_access::set_next_handle(h0_prev,h1_next_handle);
            //--- Make sure that the outgoing halfedge from vertex B is stil valid
            //--- First test if B is becoming an isolated vertex
            if(h0_prev_handle==h1_handle)
            {
              dest1->set_outgoing_halfedge_handle(this->null_halfedge_handle());
            }
            //--- Test if the edge we are deleting is the outgoing edge from B
            //--- In which case we must pick a new valid outgoing edge of B
            else if(h0_handle == dest1->get_outgoing_halfedge_handle())
            {
              dest1->set_outgoing_halfedge_handle(h1_next_handle);
            }
            //--- Finally adjust outgoing halfedge to point to empty gap in 1-ring neighborhood of B
            polymesh_core_access::adjust_outgoing_halfedge_handle(dest1);
          }
          //--- Make sure that nothing in the edges ``points'' to someting
          {
            h0->set_next_handle(this->null_halfedge_handle());
            h0->set_twin_handle(this->null_halfedge_handle());
            h0->set_destination_handle(this->null_vertex_handle());
            h0->set_edge_handle(this->null_edge_handle());

            h1->set_next_handle(this->null_halfedge_handle());
            h1->set_twin_handle(this->null_halfedge_handle());
            h1->set_destination_handle(this->null_vertex_handle());
            h1->set_edge_handle(this->null_edge_handle());

            e->set_halfedge0_handle(this->null_halfedge_handle());
            e->set_halfedge1_handle(this->null_halfedge_handle());
          }
          //--- Finally ask kernel to delete the half-edges and the edges
          {
            this->erase_edge(e->get_handle());
            this->erase_halfedge(h0_handle);
            this->erase_halfedge(h1_handle);
          }
          return true;
        }

        bool remove_face(face_handle const & f)
        {
          //--- Make sure face is valid
          if(!this->is_valid_face_handle(f))
            return false;
          //--- Unlink border edges from face, and try to delete them
          return remove_face( *this->get_face_iterator(f) );
        }

        bool remove_face(std::shared_ptr<face_type> f)
        {
          //--- Clean up border
          std::vector<std::shared_ptr<edge_type>> edges;

          for(auto h : face_halfedge_circulator(f))
          {
            polymesh_core_access::set_face_handle(h,this->null_face_handle());
            edges.push_back(*h->get_edge_iterator());
          }

          for(auto v : face_vertex_circulator(f))
          {
            vertex_iterator iter = this->get_vertex_iterator(v->get_handle());
            polymesh_core_access::adjust_outgoing_halfedge_handle( iter );
          }

          for(auto e : edges)
          {
            this->remove_edge(e);
          }

          //--- Make sure face is not ``pointing'' to something
          polymesh_core_access::set_border_halfedge_handle(f,this->null_halfedge_handle());

          //--- Ask kernel to remove face
          this->erase_face(f->get_handle());
          return true;
        }

      };

    } // namespace detail
  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_POLYMESH_MESH_H
#endif
