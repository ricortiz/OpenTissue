#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_REGULARIZATION_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_REGULARIZATION_VECTOR_H
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
  namespace mbd
  {
    namespace detail
    {
      /**
      * Extract Initial Regularization (Damping) Terms.
      * In our specific case we have A = J M^{-1} J^T and we want to alter A by
      * adding positive values to the diagonal. That is:
      *
      *   A' = A + gamma I
      *
      * This method determines a value of gamma by asking each constraint
      * in the system what values the constraint would prefer.
      *
      * This is termed constraint force mixing by the ODE community.
      *
      * The evaluate_constraints() method is supposed to be invoked prior to this method.
      *
      * Further it is assumed that the vector library used provides a vector
      * proxy function called subrange, which is capable of returning a
      * vector range. (see for instance in Boost uBLAS for an example).
      *
      * @param group        The group corresponding to the A-matrix.
      * @param m            The number of active constraints in the group (i.e. the
      *                     number of rows in the Jacobian matrix).
      * @param gamma        Upon return this vector holds the amount of damping for
      *                     each constraint variable.
      */
      template<typename group_type,typename vector_type>
      void get_regularization_vector(
        std::shared_ptr<group_type> group
        , size_t const & m
        , vector_type & gamma
        )
      {
        typedef typename group_type::math_policy   math_policy;
        typedef typename vector_type::size_type    size_type;
        typedef typename math_policy::vector_range vector_range;

        math_policy::resize( gamma, m);

        for(auto constraint : group->constraints())
        {
          if(constraint->is_active())
          {
            size_type const start = constraint->get_jacobian_index();
            size_type const end   = start + constraint->get_number_of_jacobian_rows();
            vector_range tmp_vector_range = math_policy::subrange(gamma,start,end);
            constraint->get_regularization(tmp_vector_range);
          }
        }

        for(auto contact : group->contacts())
        {
          if(contact->is_active())
          {
            size_type const start = contact->get_jacobian_index();
            size_type const end = start + contact->get_number_of_jacobian_rows();
            vector_range tmp_vector_range = math_policy::subrange(gamma,start,end);
            contact->get_regularization(tmp_vector_range);
          }
        }
      }

    } //--- End of namespace detail
  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_REGULARIZATION_VECTOR_H
#endif
