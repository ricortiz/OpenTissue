#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_JACOBIAN_MATRIX_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_JACOBIAN_MATRIX_H
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
      * Extract Jacobian.
      * The evaluate_constraints() method is supposed to be invoked prior to this method.
      *
      * Further it is assumed that the vector library used provides a vector
      * proxy function called subrange, which is capable of returning a
      * vector range. (see for instance in Boost uBLAS for an example).
      *
      * @param group        The group corresponding to the A-matrix.
      *
      * @param m            The number of active constraints in the group (i.e. the
      *                     number of rows in the Jacobian matrix).
      *
      *
      * @param J   Upon return this argument holds the Jacobian Matrix.
      */
      template<typename group_type,typename matrix_type>
      void get_jacobian_matrix(
        std::shared_ptr<group_type> const group
        , size_t const & m
        , matrix_type & J
        )
      {
        typedef typename group_type::math_policy                                math_policy;
        typedef typename math_policy::matrix_range                              matrix_range;
        typedef typename matrix_type::size_type                                 size_type;

        size_type n = group->size_bodies();

        math_policy::resize(J,m,6*n);

        J.clear();

        size_type tag = 0;
        for(auto body : group->bodies())
        {
          assert(body->is_active() || !"get_jacobian(): body was not active");
          body->m_tag = tag++;
        }


        for(auto constraint : group->constraints())
        {
          if(constraint->is_active())
          {
            size_type start_row    = constraint->get_jacobian_index();
            size_type end_row      = start_row + constraint->get_number_of_jacobian_rows();

            size_type start_column = 6*constraint->get_body_A()->m_tag;
            size_type end_column   = start_column + 3;
            matrix_range linear_matrix_range_A = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            constraint->get_linear_jacobian_A( linear_matrix_range_A );

            start_column += 3;
            end_column   += 3;
            matrix_range angular_matrix_range_A = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            constraint->get_angular_jacobian_A( angular_matrix_range_A );

            start_column = 6*constraint->get_body_B()->m_tag;
            end_column   = start_column + 3;
            matrix_range linear_matrix_range_B = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            constraint->get_linear_jacobian_B(linear_matrix_range_B);

            start_column += 3;
            end_column   += 3;
            matrix_range angular_matrix_range_B = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            constraint->get_angular_jacobian_B(angular_matrix_range_B);
          }
        }
        for(auto contact : group->contacts())
        {
          if(contact->is_active())
          {
            size_type start_row    = contact->get_jacobian_index();
            size_type end_row      = start_row + contact->get_number_of_jacobian_rows();

            size_type start_column = 6*contact->get_body_A()->m_tag;
            size_type end_column   = start_column + 3;
            matrix_range linear_matrix_range_A = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            contact->get_linear_jacobian_A( linear_matrix_range_A );

            start_column += 3;
            end_column   += 3;
            matrix_range angular_matrix_range_A = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            contact->get_angular_jacobian_A( angular_matrix_range_A );

            start_column = 6*contact->get_body_B()->m_tag;
            end_column   = start_column + 3;
            matrix_range linear_matrix_range_B = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            contact->get_linear_jacobian_B( linear_matrix_range_B );

            start_column += 3;
            end_column   += 3;
            matrix_range angular_matrix_range_B = math_policy::subrange(J,start_row,end_row,start_column,end_column);
            contact->get_angular_jacobian_B( angular_matrix_range_B );
          }
        }
      }

    } //--- end of namespace detail
  } //--- end of namespace mbd
} //--- end of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_JACOBIAN_MATRIX_H
#endif
