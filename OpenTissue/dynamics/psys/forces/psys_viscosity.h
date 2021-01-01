#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_VISCOSITY_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_VISCOSITY_H
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
  namespace psys
  {

    template<typename types>
    class Viscosity
      : public types::force_type
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::system_type           system_type;

    protected:

      real_type  m_viscosity; ///< Viscosity strength.

    public:

      real_type       & viscosity()       { return m_viscosity; }
      real_type const & viscosity() const { return m_viscosity; }

    public:

      Viscosity()
        : m_viscosity(9.81)
      {}

      Viscosity(real_type const v)
        : m_viscosity(v)
      {}

      ~Viscosity(){}

    public:

      void apply()
      {
        using std::fabs;

        if(!(fabs(m_viscosity) > 0))
          return;

        for(auto p : this->owner()->particles())
        {
          if( p->inv_mass() <= 0 )
            continue;
          p->force() -= p->velocity()*m_viscosity;
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_VISCOSITY_H
#endif
