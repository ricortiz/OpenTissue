#ifndef OPENTISSUE_COLLISION_BVH_BVH_GET_LEAF_NODES_H
#define OPENTISSUE_COLLISION_BVH_BVH_GET_LEAF_NODES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <memory> //needed for std::const_pointer_cast

namespace OpenTissue
{
  namespace bvh
  {

    /**
    *
    */
    template<typename bvh_type,typename bv_ptr_container>
    inline void get_leaf_nodes(bvh_type const & bvh,bv_ptr_container & leaves)
    {
      typedef typename bvh_type::bv_type             bv_type;
      typedef typename bvh_type::bv_ptr         bv_ptr;
      typedef typename bvh_type::bv_iterator     bv_iterator;

      leaves.clear();

      if ( !bvh.root() )
        return ;

      bv_ptr root = std::const_pointer_cast<bv_type>( bvh.root() );

      bv_ptr_container Q;
      Q.push_back( root );

      while ( !Q.empty() )
      {
        bv_ptr  bv( Q.front() ); Q.pop_front();

        if(bv->is_leaf())
          leaves.push_back( bv );
        else
        {
          for(auto &child : *bv)
          {
            Q.push_back(child);
          }
        }
      }
    }

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BVH_GET_LEAF_NODES_H
#endif
