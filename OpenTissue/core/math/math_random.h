#ifndef OPENTISSUE_CORE_MATH_MATH_RANDOM_H
#define OPENTISSUE_CORE_MATH_MATH_RANDOM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>
#include <OpenTissue/core/math/math_constants.h>

#include <random>

namespace OpenTissue {
namespace math {

template<typename value_type>
class Random
{
public:

  typedef std::minstd_rand                           generator_type;
  typedef std::random_device                         random_device_type;
  typedef std::uniform_real_distribution<value_type> distribution_type;

public:

  Random(value_type const lower = 0, value_type const upper = 1)
  {
    m_generator = generator_type(m_random_device());
    m_distribution = distribution_type(lower, upper);
  }

  Random(Random const &) = delete;
  Random & operator=(Random const &) = delete;

public:

  value_type operator()()
  {
    return m_distribution(m_generator);
  }

  bool operator==(Random const & rnd) const
  {
    return m_distribution == rnd.m_distribution;
  }

private:

  generator_type          m_generator;
  distribution_type       m_distribution;
  random_device_type      m_random_device;
};

} // namespace math
} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_RANDOM_H
#endif
