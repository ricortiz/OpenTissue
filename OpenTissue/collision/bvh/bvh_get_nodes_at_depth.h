#ifndef OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_DEPTH_H
#define OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_DEPTH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <memory> //needed for std::const_pointer_cast

#include <list>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    *
    */
    template<typename bvh_type,typename bv_ptr_container>
    inline void get_nodes_at_depth(bvh_type const & bvh, unsigned int const & depth, bv_ptr_container & nodes)
    {
      typedef typename bvh_type::bv_type           bv_type;
      typedef typename bvh_type::bv_ptr            bv_ptr;
      typedef typename bvh_type::bv_iterator   bv_iterator;

      nodes.clear();
      if ( !bvh.root() )
        return;

      bv_ptr_container Q;          //--- work queue
      std::list< unsigned int > D; //--- depth queue

      bv_ptr root = std::const_pointer_cast<bv_type>( bvh.root() );

      Q.push_back( root );
      unsigned int zero = 0;
      D.push_back( zero );

      while ( !Q.empty() )
      {
        bv_ptr bv( Q.front() ); Q.pop_front();
        unsigned int bv_depth = D.front();
        D.pop_front();

        if ( depth == bv_depth )
          nodes.push_back( bv );

        for(auto &child : *bv)
        {
          Q.push_back( child );
          D.push_back( bv_depth + 1 );
        }
      }
    }

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_DEPTH_H
#endif
