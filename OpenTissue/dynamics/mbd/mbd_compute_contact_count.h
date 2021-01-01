#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Compute Contact Count.
    * This template function computes the number of contact points
    * between a specified sequence of bodies. This is usefull for
    * debugging or profiling information.
    *
    * Example of usage:
    *
    * std::cout << "|C| = " << mbd::compute_contact_count(configuration.bodies()) << std::endl;
    *
    */
    template< typename body_ptr_container>
    size_t compute_contact_count(body_ptr_container const & bodies)
    {
      typedef typename body_ptr_container::value_type   body_type;

      size_t cnt = 0;

      for(auto body : bodies)
      {
        for(auto edge : body->edges())
        {
          if(edge->is_up_to_date() )
            cnt += edge->size_contacts();
        }
      }
      //--- Edges have been visited twice once in each
      //--- direction, so we need to divide by 2
      return (cnt/2);
    }

  } // namespace mbd

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_H
#endif
