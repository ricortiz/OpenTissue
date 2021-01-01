#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_VELOCITY_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_VELOCITY_VECTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <memory>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Set velocities and spins of a sequence of bodies.
    *
    * @param group
    * @param u       This vector holds the extracted generalized velocity vector.
    */
    template<typename group_type,typename vector_type>
    void set_velocity_vector(std::shared_ptr<group_type> group, vector_type & u)
    {
      typedef typename group_type::body_type      body_type;
      typedef typename body_type::vector3_type    vector3_type;
      typedef typename vector_type::size_type     size_type;

      vector3_type V,W;
      size_type n = group->size_bodies();

      assert(u.size() == 6*n || !"set_velocity_vector(): u has incorrect dimension");

      typename vector_type::const_iterator uval = u.begin();
      for(auto body : group->bodies())
      {
        assert(body->is_active() || !"set_velocity_vector(): body was not active");
        V(0) = *uval++;
        V(1) = *uval++;
        V(2) = *uval++;
        W(0) = *uval++;
        W(1) = *uval++;
        W(2) = *uval++;
        assert(is_number(V(0)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(V(1)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(V(2)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(W(0)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(W(1)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(W(2)) || !"set_velocity_vector(): non number encountered");
        if(!body->is_scripted())
        {
          body->set_velocity(V);
          body->set_spin(W);
        }
      }
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_VELOCITY_VECTOR_H
#endif
