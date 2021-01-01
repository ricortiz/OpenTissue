#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_POSITION_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_POSITION_VECTOR_H
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
    * Extract Generalized Position Vector.
    * Note this call also affect the tag-member of all
    * bodies in the group. Thus, in a parallel computation
    * side-effects could occur if bodies are ``shared'' among
    * different groups.
    *
    * @param group   Group body configuration
    * @param s       Upon return this vector holds the extracted
    *                generalized position vector.
    */
    template<typename group_type,typename vector_type>
    void get_position_vector(std::shared_ptr<group_type> group, vector_type & s)
    {
      typedef typename group_type::body_type      body_type;
      typedef typename body_type::math_policy     math_policy;
      typedef typename body_type::vector3_type    vector3_type;
      typedef typename body_type::quaternion_type quaternion_type;
      typedef typename vector_type::size_type     size_type;

      vector3_type r;
      quaternion_type Q;

      size_type n = group->size_bodies();

      math_policy::resize( s, 7*n);

      typename vector_type::iterator sval = s.begin();
      for(auto body : group->bodies())
      {
        assert(body->is_active() || !"get_position_vector(): body was not active");

        body->get_position(r);
        body->get_orientation(Q);

        assert(is_number(r(0))     || !"get_position_vector(): non number encountered");
        assert(is_number(r(1))     || !"get_position_vector(): non number encountered");
        assert(is_number(r(2))     || !"get_position_vector(): non number encountered");
        assert(is_number(Q.s())    || !"get_position_vector(): non number encountered");
        assert(is_number(Q.v()(0)) || !"get_position_vector(): non number encountered");
        assert(is_number(Q.v()(1)) || !"get_position_vector(): non number encountered");
        assert(is_number(Q.v()(2)) || !"get_position_vector(): non number encountered");

        *sval++ = r(0);
        *sval++ = r(1);
        *sval++ = r(2);
        *sval++ = Q.s();
        *sval++ = Q.v()(0);
        *sval++ = Q.v()(1);
        *sval++ = Q.v()(2);
      }
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_POSITION_VECTOR_H
#endif
