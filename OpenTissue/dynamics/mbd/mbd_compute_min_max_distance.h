#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_MIN_MAX_DISTANCE_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_MIN_MAX_DISTANCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/utility/utility_numeric_cast.h>

namespace OpenTissue
{
  namespace mbd
  {
    template<typename configuration_type, typename real_type2>
    void compute_min_max_distance(std::shared_ptr<configuration_type> configuration, real_type2 & min_value, real_type2 & max_value)
    {
      using std::max;
      using std::min;

      typedef typename configuration_type::body_type     body_type;
      typedef typename configuration_type::edge_type     edge_type;
      typedef typename body_type::real_type              real_type;
      typedef typename body_type::vector3_type           vector3_type;
      typedef typename body_type::matrix3x3_type         matrix3x3_type;
      typedef typename body_type::quaternion_type        quaternion_type;
      typedef typename configuration_type::edge_iterator edge_iterator;

      real_type m = OpenTissue::math::detail::highest<real_type>();
      real_type M = OpenTissue::math::detail::lowest<real_type>();

      for(auto &edge : configuration->edges())
      {
        if(edge->is_up_to_date())
        {
          for(auto contact : edge->get_contacts())
          {
            m=min(m,contact->m_distance);
            M=max(m,contact->m_distance);
          }
        }
      }

      min_value = OpenTissue::utility::numeric_cast<real_type2>(m);
      max_value = OpenTissue::utility::numeric_cast<real_type2>(M);
    }

  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_MIN_MAX_DISTANCE_H
#endif
