#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_ANALYSIS_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_ANALYSIS_H
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
    /**
    * Stack Analysis.
    *
    * This auxiliary tool analyses a contact group in order to see if it forms a ``stack''.
    *
    * A stack is loosely defined as when something is in contact with a fixed body.
    *
    * Contacts (and constraints) can be assigned a stack layer, equivalent to
    * how long a path there is to the closest fixed body.
    *
    * The two incident bodies at a contact can be assigned a lower or upper state. Depending
    * on which one that is closest to a fixed body. This state can be deduced from the stack
    * height of the bodies. Stack height is the smallest number of contacts to a fixed body.
    *
    */
    template<typename mbd_types>
    class StackAnalysis
    {
    protected:

      typedef typename mbd_types::math_policy::index_type    index_type;
      typedef typename mbd_types::math_policy::index_type    size_type;

      typedef typename mbd_types::math_policy::real_type     real_type;
      typedef typename mbd_types::math_policy::vector3_type  vector3_type;
      typedef typename mbd_types::group_type                 group_type;
      typedef typename mbd_types::body_type                  body_type;
      typedef typename mbd_types::edge_type                  edge_type;

      typedef typename mbd_types::contact_type              contact_type;
      typedef typename mbd_types::material_type             material_type;
      typedef typename mbd_types::edge_ptr_container        edge_ptr_container;
      typedef typename mbd_types::group_ptr_container       group_ptr_container;

    protected:

      typedef typename std::vector<std::shared_ptr<contact_type>>             contact_ptr_heap;
      typedef typename std::list<std::shared_ptr<body_type>>                  body_ptr_queue;

    public:

      class node_traits
      {
      public:
        size_type m_sa_stack_height;   ///< The number of bodies way from closest fixed body. Fixed bodies have height zero.
        bool      m_sa_queue_tag;      ///< Boolean flag set to true if node when node have been  pushed into queue.

        node_traits()
          : m_sa_stack_height(0)
        {}

      };

      class edge_traits
      {
      public:
        size_type m_sa_stack_layer;       ///< Equal to the minimum stack height of incident nodes.
        size_type m_sa_visit_time_stamp;  ///< Time stamp indicating last time the edge was traversed.

        edge_traits()
          : m_sa_visit_time_stamp(0)
        {}
      };

      class constraint_traits{ };

    protected:

      index_type  m_time_stamp;         ///< Time-stamp of last time algorithm was invoked.

    public:

      StackAnalysis()
        : m_time_stamp(0)
      {}

    public:

      /**
      * Analyse Contact group_type.
      *
      * @param group    The contact group that should be analysed.
      * @param layers   Upon return this argument holds the stack layers, they are stored in increasing height.
      *
      * @return         The number of stack layers, this is equal to the maximum height of the stack.
      */
      size_type analyze(std::shared_ptr<group_type> group,group_ptr_container & layers)
      {
        layers.clear();

        size_type N = group->size_bodies();
        if(N==0)
          return 0;

        ++m_time_stamp;
        body_ptr_queue Q;
        {
          for(auto body : group->bodies())
          {
            if(body->is_fixed() || body->is_scripted())
            {
              body->m_sa_stack_height = 0;
              Q.push_back(body);
              body->m_sa_queue_tag = true;
            }
            else
            {
              body->m_sa_stack_height = N;
              body->m_sa_queue_tag = false;
            }
          }
        }
        if(Q.empty() || (group->size_contacts()==0))
        {
          //--- No fixed objects, just return the original contact group as result...
          layers.resize(1);
          layers[0] = group;
          return 1;
        }

        edge_ptr_container edges;  //--- Used to keep pointers to all edges in contact group
        size_type height = 0; //--- Used to figure out how many layers there is in the stack
        while(!Q.empty())
        {
          auto body = Q.front();
          Q.pop_front();
          for(auto edge : body->edges())
          {
            if(!edge->is_up_to_date())//--- Make sure we only process edges that contain valid cached information.
              continue;

            auto next = (body==edge->get_body_A())? edge->get_body_B(): edge->get_body_A();
            bool has_joint = body->has_joint_to(next);
            if(edge->size_contacts()==0 && !has_joint)//--- Make sure we only process edges that contain contact information
              continue;

            if(!next->m_sa_queue_tag)
            {
              Q.push_back(next);
              next->m_sa_queue_tag = true;
            }

            next->m_sa_stack_height = std::min(next->m_sa_stack_height,body->m_sa_stack_height + 1);

            if(next->m_sa_stack_height == body->m_sa_stack_height && next->m_sa_stack_height!=0)
              edge->m_sa_stack_layer = next->m_sa_stack_height-1;
            else
              edge->m_sa_stack_layer = std::min(next->m_sa_stack_height,body->m_sa_stack_height);

            height = std::max(height,edge->m_sa_stack_layer);

            if(edge->m_sa_visit_time_stamp!=m_time_stamp)
            {
              edge->m_sa_visit_time_stamp=m_time_stamp;
              edges.push_back(edge);
            }
          }
        }
        if(edges.empty())
        {
          //--- This means that the fixed objects were not in contact with any non-fixed objects...
          layers.resize(1);
          layers[0] = group;
          return 1;
        }
        //--- Now simply traverse edges and build stack layers
        {
          layers.resize(height+1);
          for(auto edge : edges)
          {
            size_type layer = edge->m_sa_stack_layer;

            for(auto contact : edge->get_contacts())
            {
              layers[layer].m_contacts.push_back(contact);
            }

            if(edge->get_body_A()->has_joint_to(edge->get_body_B()))
            {
              for(auto joint : edge->get_body_A()->joints())
              {
                if(joint->get_socket_A()->get_body()==edge->get_body_B() || joint->get_socket_B()->get_body()==edge->get_body_B())
                {
                  layers[layer].m_constraints.push_back(joint);
                }
              }
            }
          }
        }
        //--- Now traverse bodies and add them to the layers
        {
          for(auto body : group->bodies())
          {
            if(body->m_sa_stack_height==N)
            {
              //--- somehow this is a non-fixed body that was not in contact with anything else, we simply add it to top layer...
              layers[height].m_bodies.push_back(&(*body));
              continue;
            }
            bool in_lower = false;
            bool in_upper = false;
            for(auto edge : body->edges())
            {
              if(!edge->is_up_to_date())//--- Make sure we only process edges that contain valid cached information.
                continue;
              auto other = edge->get_body_A()==body?edge->get_body_B():edge->get_body_A();
              if(other->m_sa_stack_height==N)//--- Bodies are not in contact, so ignore this edge
                continue;
              if(other->m_sa_stack_height > body->m_sa_stack_height)
                in_upper = true;
              if(other->m_sa_stack_height < body->m_sa_stack_height)
                in_lower = true;
              if(in_upper && in_lower)
                break;
            }
            if(in_upper)
              layers[body->m_sa_stack_height].m_bodies.push_back(body);
            if(in_lower)
              layers[body->m_sa_stack_height - 1].m_bodies.push_back(body);
          }
        }
        return height+1;
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_STACK_ANALYSIS_H
#endif
