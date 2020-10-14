#ifndef OPENTISSUE_CORE_MATH_MATH_TRILLINEAR_H
#define OPENTISSUE_CORE_MATH_MATH_TRILLINEAR_H
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

  namespace math
  {

    /**
    * Trillinear Interpolation.
    * Interpolation of values in a cubic grid.
    */
    template <typename T,typename T2>
    inline T  trillinear(
      T const & d000
      , T const & d001
      , T const & d010
      , T const & d011
      , T const & d100
      , T const & d101
      , T const & d110
      , T const & d111
      , T2 const & s
      , T2 const & t
      , T2 const & u
      )
    {
      T x00 = (d001 - d000) * s + d000;
      T x01 = (d011 - d010) * s + d010;
      T x10 = (d101 - d100) * s + d100;
      T x11 = (d111 - d110) * s + d110;
      T y0  = ( x01 -  x00) * t +  x00;
      T y1  = ( x11 -  x10) * t +  x10;
      return (y1 - y0) * u + y0;
    }

  } // namespace math

} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_MATH_TRILLINEAR_H
#endif
