#ifndef OPENTISSUE_BVH_BVH_SINGLE_COLLISION_QUERY_H
#define OPENTISSUE_BVH_BVH_SINGLE_COLLISION_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <memory> // needed for std::const_pointer_cast

namespace OpenTissue
{
namespace bvh
{

  /**
  * Single Collision Query.
  * This query tests the BVH recursively against a user specified geometry.
  */
  template <typename collision_policy>
  class SingleCollisionQuery : public collision_policy
  {
  public:

    /**
    * Collision Query.
    *
    * @param xform     Coordinate transform, can be used to bring bvh into geometry frame or vice versa.
    * @param bvh       The bvh.
    * @param geometry  The geometry.
    * @param results   Upon return this container contains any results from the
    *                  collision query.
    */
    template<typename coordsys_type,typename bvh_type, typename user_geometry_type,typename results_container>
    void run(
        coordsys_type const & xform,
        bvh_type const & bvh,
        user_geometry_type const & geometry,
        results_container & results
    )
    {
      typedef typename bvh_type::bv_type                  bv_type;
      typedef typename bvh_type::bv_ptr                   bv_ptr;
      typedef typename bvh_type::bv_ptr_container         bv_ptr_container;
      typedef typename bvh_type::bv_iterator          bv_iterator;

      collision_policy::reset(results);//--- collision_policy

      bv_ptr_container Q;

      bv_ptr root = std::const_pointer_cast<bv_type>( bvh.root() );

      Q.push_back( root );

      while ( !Q.empty() )
      {
        bv_ptr bv( Q.front() ); Q.pop_front();
        if( !this->overlap( xform, bv, geometry ) )  //--- collision_policy
          continue;
        if ( bv->is_leaf() )
        {
          this->report( xform, bv, geometry, results ); //--- collision_policy
          continue;
        }
        for(auto &child : *bv)
        {
          Q.push_back( child );
        }
      }
    }

  };

} // namespace bvh
} // namespace OpenTissue

// OPENTISSUE_BVH_BVH_SINGLE_COLLISION_QUERY_H
#endif
