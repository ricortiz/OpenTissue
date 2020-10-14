#ifndef OPENTISSUE_COLLISION_BVH_BVH_BV_NODE_H
#define OPENTISSUE_COLLISION_BVH_BVH_BV_NODE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>

namespace OpenTissue
{
  namespace bvh
  {

    template <typename V, typename G,typename T>  class BoundingVolumeHierarchy; ///< Forward Declaration

    /**
    * BV Node Class.
    * A non-annotated BV, implying that no geometry
    * can be stored in the BV.
    *
    * The first template argument is the bounding volume
    * hierarchy type and the second template argument is
    * a user defined bounding volume traits class.
    */
    template <typename B, typename  T>
    class BV : public T
    {
    public:

      typedef B                                         bvh_type;
      typedef typename bvh_type::volume_type            volume_type;
      typedef typename bvh_type::geometry_type          geometry_type;

      typedef typename bvh_type::bvh_weak_ptr           bvh_weak_ptr;
      typedef typename bvh_type::bvh_const_weak_ptr     bvh_const_weak_ptr;
      typedef typename bvh_type::bvh_const_ptr          bvh_const_ptr;
      typedef typename bvh_type::bv_weak_ptr            bv_weak_ptr;
      typedef typename bvh_type::bv_const_weak_ptr      bv_const_weak_ptr;
      typedef typename bvh_type::bv_const_ptr           bv_const_ptr;

      typedef typename bvh_type::bv_ptr_container       bv_ptr_container;
      typedef typename bvh_type::bv_iterator            bv_iterator;
      typedef typename bvh_type::bv_const_iterator      bv_const_iterator;

    public:

      friend class BoundingVolumeHierarchy<volume_type,geometry_type,T>;

    protected:

      bvh_weak_ptr     m_owner;              ///< Owner Pointer.
      bv_weak_ptr      m_parent;             ///< Parent Pointer.
      bv_ptr_container m_children;           ///< List of children Pointers.
      volume_type      m_volume;             ///< Volume type.
      bool             m_has_geometry;       ///< Boolean value, set to true if BV stores geometry.

    public:

      BV()
        : m_owner()
        , m_parent()
        , m_children()
        , m_volume()
        , m_has_geometry(false)
      {}

    public:

      bv_iterator       begin()       { return m_children.begin(); }
      bv_iterator       end()         { return m_children.end();   }
      bv_const_iterator begin() const { return m_children.begin(); }
      bv_const_iterator end()   const { return m_children.end();   }

    public:

      size_t const size()     const { return m_children.size(); }
      size_t const children() const { return this->size(); }
      size_t const degree()   const { return this->size(); }

      bvh_const_ptr owner()  const { return m_owner.lock(); }
      bv_const_ptr  root()   const { return m_owner.lock()->root(); }
      bv_const_ptr  parent() const { return m_parent.lock(); }

      volume_type const & volume() const { return m_volume; }
      volume_type       & volume()       { return m_volume; }

      bool is_leaf()      const { return ( m_children.empty() ); }
      bool is_root()      const { assert( !m_owner.expired() ); return ( m_parent.expired() ); }
      bool has_geometry() const { return m_has_geometry; }

    };

  } // namespace bvh
} // namespace OpenTissue

template<typename B, typename T>
typename OpenTissue::bvh::BV<B,T>::bv_iterator begin(OpenTissue::bvh::BV<B,T> &bv)
{
  return bv.begin();
}

template<typename B, typename T>
typename OpenTissue::bvh::BV<B,T>::bv_const_iterator begin(const OpenTissue::bvh::BV<B,T> &bv)
{
  return bv.begin();
}

template<typename B, typename T>
typename OpenTissue::bvh::BV<B,T>::bv_iterator end(OpenTissue::bvh::BV<B,T> &bv)
{
  return bv.end();
}

template<typename B, typename T>
typename OpenTissue::bvh::BV<B,T>::bv_const_iterator end(const OpenTissue::bvh::BV<B,T> &bv)
{
  return bv.end();
}

//OPENTISSUE_COLLISION_BVH_BVH_BV_NODE_H
#endif
