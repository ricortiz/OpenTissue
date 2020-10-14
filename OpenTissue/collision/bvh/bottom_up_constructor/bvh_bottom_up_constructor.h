#ifndef OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_BOTTOMUP_CONSTRUCTOR_H
#define OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_BOTTOMUP_CONSTRUCTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_graph.h>
#include <OpenTissue/collision/bvh/bottom_up_constructor/bvh_default_priority_bottom_up_policy.h>

#include <memory>
#include <iostream>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * A graph-based deterministic bottom-up BVH contruction algorithm
    *
    * The bottom-up policy has to implement the following methods:
    *
    *   void init(graph_type & graph)
    *   void update_priorities(node_type * node)
    *   edge_type * next_edge(void)const
    *   const bool more_edges(void)const
    *   bool should_create_bv(node_type * node)
    *   volume_type fit(geometry_iterator g0,geometry_iterator g1,volume_iterator v0,volume_iterator v1)
    *
    * See class DefaultPriorityBottomUpPolicy for details about what these methods are supposed to do.
    */
    template <
      typename bvh_type,
      typename bottom_up_policy = DefaultPriorityBottomUpPolicy<bvh_type>
    >
    class BottomUpConstructor : public bottom_up_policy
    {
    public:

      //--- Convenience stuff for better readability
      typedef typename bvh_type::volume_type            volume_type;
      typedef typename bvh_type::geometry_type          geometry_type;
      typedef typename bvh_type::bv_ptr                 bv_ptr;
      typedef typename bvh_type::annotated_bv_ptr       annotated_bv_ptr;
      typedef typename bvh_type::annotated_bv_type      annotated_bv_type;
      typedef BVHGraph<bvh_type>                        graph_type;
      typedef typename graph_type::edge_ptr_type        edge_ptr;
      typedef typename graph_type::node_ptr_type        node_ptr;
      typedef typename graph_type::node_iterator        node_iterator;
      typedef typename graph_type::volume_container     volume_container;

    public:

      /**
      * Run Algorithm.
      *
      * @param graph      Initial graph, nodes represent leaves in resulting BVH, and
      *                   edges represent possible ways to group BV nodes into parent
      *                   nodes during the bottom-up construction.
      * @param bvh        Upon return this argument holds the resulting BVH.
      *
      */
      void run(graph_type & graph, bvh_type & bvh)
      {
        bvh.clear();
        //--- Create leaf BVs in BVH

        for(auto &node : graph.nodes())
        {
          if(node->coverage().empty())
          {
            node->create_bv(bvh);
          }
          else
          {
            node->create_bv(bvh, true);
            annotated_bv_ptr bv = std::static_pointer_cast<annotated_bv_type>(node->bv());

            bv->insert(node->coverage());

            volume_container volumes; //--- empty!

            bv->volume() = bottom_up_policy::fit(node->coverage().begin(),node->coverage().end(),volumes.begin(),volumes.end());
          }
        }

        bottom_up_policy::init(graph);//--- from policy

        //--- Begin to merge leaves into parents, until only a single root exist
        while(this->has_more_edges())
        {
          //--- pick an graph edge and collapse it
          edge_ptr edge = this->get_next_edge();//--- from policy
          node_ptr node = graph.collapse(edge);
          //--- Fit a volume to the new graph node
          volume_container volumes;
          node->get_volumes(volumes);
          node->volume() = bottom_up_policy::fit(node->coverage().begin(),node->coverage().end(),volumes.begin(),volumes.end());
          //--- Test if a BV node should be created for the new graph node
          if( bottom_up_policy::should_create_bv(node) )  //--- from policy
          {
            node->create_bv(bvh);
            graph.remove_sub_nodes(node);
          }
          bottom_up_policy::update(node);//--- from policy
        }
        graph.clear();
        std::cout << "BVHBottomUpConstructor::run(): " << bvh.size() << " nodes created." << std::endl;
      }

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_BOTTOMUP_CONSTRUCTOR_H
#endif
