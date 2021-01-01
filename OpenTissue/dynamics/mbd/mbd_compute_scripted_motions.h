#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_SCRIPTED_MOTIONS_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_SCRIPTED_MOTIONS_H
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
    * Compute Scripted Motion.
    * This method will update the scripted motion of all scripted bodies
    * in the body sequence. That is the state of each scripted body is updated by
    * invoking the run method on the attached scripted motion.
    *
    *
    * @param group
    * @param time       The time at which the scripted motion should be evaluated.
    */
    template<typename group_type,typename real_type>
    void compute_scripted_motions(std::shared_ptr<group_type> group, real_type const & time)
    {
      for(auto body : group->bodies())
      {
        assert(body->is_active() || !"get_position_vector(): body was not active");
        if(body->is_scripted())
        {
          body->compute_scripted_motion(time);
        }
      }
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_SCRIPTED_MOTIONS_H
#endif
