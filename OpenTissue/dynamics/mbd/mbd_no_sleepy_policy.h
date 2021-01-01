#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_NO_SLEEPY_POLICY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_NO_SLEEPY_POLICY_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * No Sleepy Policy.
    * This is an implementation of a default sleepy
    * policy
    */
    template< typename mbd_types>
    struct NoSleepyPolicy
    {
      typedef typename mbd_types::group_type          group_type;
      typedef typename group_type::body_ptr_container body_ptr_container;

      class node_traits{};
      class edge_traits{};
      class constraint_traits{};

      void evaluate(body_ptr_container &){}

      void clear(){}
    };

  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_NO_SLEEPY_POLICY_H
#endif
