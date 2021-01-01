#ifndef OPENTISSUE_DYNAMICS_MBD_BODY_GROUP_H
#define OPENTISSUE_DYNAMICS_MBD_BODY_GROUP_H
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

    template< typename mbd_types >
    class BodyGroup
    {
    public:

      typedef typename mbd_types::math_policy               math_policy;
      typedef typename mbd_types::body_type                 body_type;
      typedef typename mbd_types::body_ptr_container        body_ptr_container;

    protected:

      typedef typename mbd_types::math_policy::index_type   size_type;
      typedef typename mbd_types::contact_ptr_container     contact_ptr_container;
      typedef typename mbd_types::constraint_ptr_container  constraint_ptr_container;

    public:  // TODO should be protected!

      contact_ptr_container      m_contacts;          ///< Container of all current constraints in the group.
      constraint_ptr_container   m_constraints;       ///< Container of all current constraints in the group.
      body_ptr_container         m_bodies;            ///< Container of all currently active ``physical'' bodies in the group.

    public:

      BodyGroup(){}

      ~BodyGroup(){ clear(); }

    public:

      size_type size_bodies()   const { return m_bodies.size();   }
      size_type size_contacts() const { return m_contacts.size(); }

      contact_ptr_container           &contacts()          { return m_contacts; }
      contact_ptr_container    const  &contacts()    const { return m_contacts; }
      constraint_ptr_container        &constraints()       { return m_constraints; }
      constraint_ptr_container const  &constraints() const { return m_constraints; }
      body_ptr_container              &bodies()            { return m_bodies; }
      body_ptr_container       const  &bodies()      const { return m_bodies; }

      void clear()
      {
        m_bodies.clear();
        m_constraints.clear();
        m_contacts.clear();
      }

    };

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_BODY_GROUP_H
#endif
