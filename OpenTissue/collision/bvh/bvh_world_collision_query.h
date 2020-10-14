#ifndef OPENTISSUE_BVH_BVH_WORLD_COLLISION_QUERY_H
#define OPENTISSUE_BVH_BVH_WORLD_COLLISION_QUERY_H
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
  namespace bvh
  {
    /**
    * World Frame Query.
    * Implicitly assumes that A and B are representated in same
    * frame (we call it world frame, but it could be any frame).
    *
    * This is typcally the kind of query that is needed for
    * testing one deformable object against another deformable
    * object.
    */
    template <typename collision_policy>
    class WorldCollisionQuery : public collision_policy
    {
    public:

      /**
      * Run Query Algorithm.
      *
      * @param bvh_A     bvh A.
      * @param bvh_B     bvh B
      * @param results   Upon return this container contains any results from the
      *                  collision query.
      */
      template<typename bvh_type, typename results_container>
      void run( bvh_type const & bvh_A, bvh_type const & bvh_B, results_container & results )
      {
        typedef typename bvh_type::bv_type              bv_type;
        typedef typename bvh_type::bv_ptr          bv_ptr;
        typedef typename bvh_type::bv_ptr_container     bv_ptr_container;
        typedef typename bvh_type::bv_iterator      bv_iterator;

        this->reset(results);//--- collision_policy

        bv_ptr_container Q;
        bv_ptr root_A =  std::const_pointer_cast<bv_type>( bvh_A.root() );
        bv_ptr root_B =  std::const_pointer_cast<bv_type>( bvh_B.root() );


        Q.push_back( root_A );
        Q.push_back( root_B );
        while ( !Q.empty() )
        {
          bv_ptr A( Q.front() );
          Q.pop_front();
          bv_ptr B( Q.front() );
          Q.pop_front();
          if( !this->overlap( A, B ) )//--- collision_policy
            continue;
          if ( A->is_leaf() && B->is_leaf() )
          {
            this->report(A,B,results); //--- collision_policy
            continue;
          }
          if (  B->is_leaf()  || ( !A->is_leaf() && (   A->volume().volume() > B->volume().volume()  )  ) )
          {
            for(auto &child : *A)
            {
              Q.push_back( child );
              Q.push_back( B );
            }
          }
          else
          {
            for(auto &child : *B)
            {
              Q.push_back( A );
              Q.push_back( child );
            }
          }
        }
      }
    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_BVH_BVH_WORLD_COLLISION_QUERY_H
#endif
