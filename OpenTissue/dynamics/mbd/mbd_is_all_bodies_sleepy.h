#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_IS_ALL_BODIES_SLEEPY_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_IS_ALL_BODIES_SLEEPY_H
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
    template<typename body_ptr_container>
    bool all_bodies_sleepy(body_ptr_container const & bodies)
    {
      for(auto body : bodies)
      {
        assert(body->is_active() || !"is_all_bodies_sleepy(): body was not active");
        if(!body->is_sleepy())
          return false;
      }
      return true;
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_IS_ALL_BODIES_SLEEPY_H
#endif
