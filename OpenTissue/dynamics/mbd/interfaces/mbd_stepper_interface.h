#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_STEPPER_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_STEPPER_INTERFACE_H
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

    template< typename mbd_types  >
    class StepperInterface
    {
    protected:

      typedef typename mbd_types::configuration_type             configuration_type;
      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::group_type                     group_type;

    protected:

      std::shared_ptr<configuration_type> m_configuration;   ///< Pointer to configuration (or sub-part) that
                                              ///< stepper works on. Can be used to access
                                              ///< information about materials etc..
    public:

      StepperInterface()
        : m_configuration(0)
      {}

      virtual ~StepperInterface(){}

    public:


      void connect(std::shared_ptr<configuration_type> configuration)
      {
        m_configuration = configuration;
      }

      void clear()
      {
        this->m_configuration = nullptr;
      }

    public:

      virtual void resolve_collisions(std::shared_ptr<group_type> group) = 0;
      virtual void error_correction(std::shared_ptr<group_type> group) = 0;
      virtual void run(std::shared_ptr<group_type> group,real_type const & time_step) = 0;

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_STEPPER_INTERFACE_H
#endif
