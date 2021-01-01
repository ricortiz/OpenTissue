#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_CONFIGURATION_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_CONFIGURATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//


#include <OpenTissue/configuration.h>

#include <unordered_map>
#include <memory>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types >
    class Configuration
    {
    public:

      typedef typename mbd_types::body_type                      body_type;
      typedef typename mbd_types::edge_type                      edge_type;
      typedef typename mbd_types::material_library_type          material_library_type;
      typedef typename mbd_types::math_policy::index_type        size_type;

    protected:

      typedef typename mbd_types::math_policy::real_type         real_type;
      typedef typename mbd_types::joint_type                     joint_type;
      typedef typename mbd_types::group_type                     group_type;
      typedef typename mbd_types::collision_detection_policy     collision_detection_policy;
      typedef typename mbd_types::body_ptr_container             body_ptr_container;
      typedef typename mbd_types::edge_ptr_container             edge_ptr_container;
      typedef typename mbd_types::joint_ptr_container            joint_ptr_container;

      typedef typename std::unordered_map<size_type, size_t>   body_ptr_lut_type;
      typedef typename std::unordered_map<size_type, size_t>   joint_ptr_lut_type;
      typedef typename std::unordered_map<size_type, size_t>   edge_lut_type;

    protected:

      std::shared_ptr<collision_detection_policy> m_collision_detection;  ///< Collision Detection Engine.
      std::shared_ptr<material_library_type>      m_material_library;     ///< material_type library
      std::shared_ptr<group_type>                 m_all;                  ///< Internal data strucure used by the method ``get_all_body_group()''
      body_ptr_container                          m_bodies;               ///< All bodies currently added to the simulator.
      body_ptr_lut_type                           m_bodies_lut;           ///< All bodies currently added to the simulator. Copy lut.
      joint_ptr_container                         m_joints;               ///< All joints currently added to the simulator.
      joint_ptr_lut_type                          m_joints_lut;           ///< All joints currently added to the simulator. Copy lut.
      edge_ptr_container                          m_edges;                ///< All edges that currently exist in the simulator.
      edge_lut_type                               m_edges_lut;            ///< All edges that currently exist in the simulator.
      real_type                                   m_collision_envelope;   ///< Size of collision envelope

    protected:

      typedef typename body_ptr_container::iterator      body_iterator;
      typedef typename joint_ptr_container::iterator      joint_iterator;

    public:

      typedef typename edge_lut_type::iterator           edge_iterator;

      body_iterator body_begin() { return m_bodies.begin(); }
      body_iterator body_end()   { return m_bodies.end(); }

      edge_iterator edge_begin() { return m_edges.begin(); }
      edge_iterator edge_end()   { return m_edges.end(); }

      joint_iterator joint_begin() { return m_joints.begin(); }
      joint_iterator joint_end()   { return m_joints.end(); }

      body_ptr_container        &bodies()       { return m_bodies; }
      body_ptr_container const  &bodies() const { return m_bodies; }
      joint_ptr_container       &joints()       { return m_joints; }
      joint_ptr_container const &joints() const { return m_joints; }
      edge_ptr_container        &edges()        { return m_edges; }
      edge_ptr_container const  &edges()  const { return m_edges; }

      size_type size_bodies() const {return m_bodies.size(); }

    public:

      Configuration()
        : m_collision_detection(nullptr)
        , m_material_library(nullptr)
        , m_collision_envelope(0.01)
      {
        m_all = std::make_shared<group_type>();
      }

      ~Configuration()
      {
        clear();
      }

    public:

      void set_material_library(std::shared_ptr<material_library_type> library)
      {
        this->m_material_library = library;
      }

      std::shared_ptr<material_library_type> get_material_library() const
      {
        return this->m_material_library;
      }

      void set_collision_envelope(real_type const & value)
      {
        assert(value>0 || !"Configuration::set_collision_envelope(): envelope must be positive");
        m_collision_envelope = value;
      }

      real_type const & get_collision_envelope() const
      {
        return m_collision_envelope;
      }

      /**
      * Collision Detection Connect Method.
      * Used by simulator class to connect configuration with collision
      * detection engine.
      *
      * @param collision_detection           The Collision Detection Engine.
      */
      void connect(std::shared_ptr<collision_detection_policy> collision_detection)
      {
        if(m_collision_detection)
          m_collision_detection->clear();

        m_collision_detection = collision_detection;

        for(auto &body : m_bodies)
        {
          m_collision_detection->add(body);
        }
      }

    public:

      /**
      * Get All body_type group_type.
      * This method sets us a body group, containing (ONLY) all currently
      * active bodies in the configuration at the time of invocation.
      *
      * @return       A pointer to a BodyGroup, containting all active bodies.
      */
      std::shared_ptr<group_type> get_all_body_group()
      {
        m_all->clear();
        for(auto &body : m_bodies)
        {
          if(body->is_active())
            m_all->m_bodies.push_back(body);
        }
        return m_all;
      }

      bool add(std::shared_ptr<body_type> body)
      {
        assert(body || !"Configuration::add(): body was null");
        assert(m_bodies_lut.find(body->get_index())==m_bodies_lut.end() || !"Configuration::add(): body was already in configuration");

        m_bodies.push_back(body);
        m_bodies_lut.insert( std::make_pair(body->get_index(), m_bodies.size()-1) );

        if(m_collision_detection)
          m_collision_detection->add(body);

        return true;
      }

      bool add(std::vector<std::shared_ptr<body_type>> &bodies)
      {
        for(auto body : bodies)
        {
          this->add(body);
        }
      }

      bool remove(std::shared_ptr<body_type> body)
      {
        assert(body || !"Configuration::remove(): body was null");
        assert(m_bodies_lut.find(body->get_index())!=m_bodies_lut.end() || !"Configuration::remove(): body was not in configuration");

        if(body->has_joints())
        {
          std::cout << "Configuration::remove(): Remove all joints on body first" << std::endl;
          return false;
        }

        size_t body_local_index = m_bodies_lut[body->get_index()];
        auto it = std::next(m_bodies.begin(), body_local_index);

        if(*it != body)
        {
          // TODO: Print error message.
          return false;
        }

        while(!body->edges().empty())
        {
          auto edge = body->edges().front();
          edge->get_body_A()->edges().remove(edge);
          edge->get_body_B()->edges().remove(edge);
          m_edges.erase(std::next(m_edges.begin(), m_edges_lut[edge->hash_key()]));
          m_edges_lut.erase(edge->hash_key());
        }

        m_bodies_lut.erase(body->get_index());
        m_bodies.erase(it);
        if(m_collision_detection)
          m_collision_detection->remove(body);
        return true;
      }

      bool add(std::shared_ptr<joint_type> joint)
      {
        assert(joint || !"Configuration::add(): Joint was null");
        assert(m_joints_lut.find(joint->get_index())==m_joints_lut.end() || !"Configuration::add(): Joint was already in configuration");
        if(joint->get_socket_A()==0)
        {
          std::cout << "Configuration::add(): Joint were missing socket A" << std::endl;
          return false;
        }
        if(joint->get_socket_A()->get_body()==0)
        {
          std::cout << "Configuration::add(): Joint socket A were missing a body" << std::endl;
          return false;
        }
        if(joint->get_socket_B()==0)
        {
          std::cout << "Configuration::add(): Joint were missing socket B" << std::endl;
          return false;
        }
        if(joint->get_socket_B()->get_body()==0)
        {
          std::cout << "Configuration::add(): Joint socket B were missing a body" << std::endl;
          return false;
        }
        auto itA = m_bodies_lut.find(joint->get_socket_A()->get_body()->get_index());
        if(itA==m_bodies_lut.end())
        {
          std::cout << "Configuration::add(): First add body on socket A" << std::endl;
          return false;
        }
        auto itB = m_bodies_lut.find(joint->get_socket_B()->get_body()->get_index());
        if(itB==m_bodies_lut.end())
        {
          std::cout << "Configuration::add(): First add body on socket B" << std::endl;
          return false;
        }

        m_joints.push_back(joint);
        m_joints_lut.insert( std::make_pair( joint->get_index(), m_joints.size()- 1 ) );
        return true;
      }

      bool remove(std::shared_ptr<joint_type> joint)
      {
        assert(joint || !"Configuration::remove(): Joint was null");
        assert(m_joints_lut.find(joint->get_index())!=m_joints_lut.end() || !"Configuration::remove(): Joint was not in configuration");
        assert( !(joint->get_body_A()) || !"Configuration::remove(): Body A on joint was non-null");
        assert( !(joint->get_body_B()) || !"Configuration::remove(): Body B on joint was non-null");

        size_t joint_local_index = m_joints_lut[joint->get_index()];
        auto it = std::next(m_joints.begin(), joint_local_index);

        if(*it != joint)
        {
          // TODO: Print error message.
          return false;
        }

        m_joints_lut.erase(joint->get_index());
        m_joints.erase(it);

        return true;
      }

      std::shared_ptr<edge_type> get_edge(std::shared_ptr<body_type> A,std::shared_ptr<body_type> B)
      {
        assert(A    || !"Configuration::get_edge(): body A was null");
        assert(B    || !"Configuration::get_edge(): body B was null");
        assert(A!=B || !"Configuration::get_edge(): body A and B were the same");

        auto edge = m_edges_lut.find(edge_type::hash_key(A,B));
        if(edge==m_edges_lut.end())
          return nullptr;

        auto it = std::next(m_edges.begin(), edge->second);

        return *it;
      }

      /**
      * Add edge_type.
      *
      * @param A     A pointer to one of the incident bodies of the edge.
      * @param B     A pointer to the other incident body of the edge.
      *
      * @return      A pointer to the newly added edge.
      */
      std::shared_ptr<edge_type> add(std::shared_ptr<body_type> A,std::shared_ptr<body_type> B)
      {
        assert(A    || !"Configuration::add(): body A was null");
        assert(B    || !"Configuration::add(): body B was null");
        assert(A!=B || !"Configuration::add(): body A and B were the same");
        assert(m_material_library || !"Configuration::add(): Material library was null");
        assert(m_edges_lut.find(edge_type::hash_key(A,B))==m_edges_lut.end() || !"Configuration::add(): Edge allready existed in configuration");
        assert(m_collision_detection || !"Configuration::add(): Collision Detection was null");

        auto edge = std::make_shared<edge_type>();
        edge->init(A,B);
        edge->get_body_A()->m_edges.push_back(edge);
        edge->get_body_B()->m_edges.push_back(edge);

        edge->m_material = m_material_library->get(A->get_material_idx(),B->get_material_idx());

        if(!edge->m_material)
        {
          std::cout << "Configuration::add(): Could not find a material between ("<< A->get_material_idx() <<","<< B->get_material_idx() <<") using default material" << std::endl;
          edge->m_material = m_material_library->default_material();
        }

        edge->m_collision_detection = m_collision_detection;
        m_edges.push_back(edge);
        m_edges_lut.insert( std::make_pair(edge_type::hash_key(A,B),m_edges.size() - 1) );

        return edge;
      }

      bool remove(std::shared_ptr<edge_type> edge)
      {
        assert(edge || !"Configuration::remove(): edge was null");
        assert(m_edges_lut.find( edge->hash_key() )!=m_edges_lut.end() || !"Configuration::remove(): edge was not in configuration");

        size_t edge_local_index = m_edges_lut[edge->hash_key()];
        auto it = std::next(m_edges.begin(), edge_local_index);

        if(*it != edge)
        {
          // TODO: Print error message.
          return false;
        }

        edge->get_body_A()->m_edges.remove(edge);
        edge->get_body_B()->m_edges.remove(edge);

        m_edges.erase(it);
        m_edges_lut.erase(edge->hash_key());

        return true;
      }

      void clear()
      {
        m_collision_detection.reset();
        m_material_library.reset();

        for(auto joint : m_joints)
        {
          joint->clear();
          remove(joint);
        }

        for(auto body : m_bodies)
        {
          remove(body);
          body->clear();
        }

        assert(m_bodies.empty() || !"Configuration::clear(): Internal error, not all bodies where removed?");
        assert(m_joints.empty() || !"Configuration::clear(): Internal error, not all joints where removed?");

        // Note that edges has been removed implicitly. This happens
        // while the bodies are removed. See method remove(body)
        assert(m_edges.empty() || !"Configuration::clear(): Internal error, not all edges where removed?");
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_CONFIGURATION_H
#endif
