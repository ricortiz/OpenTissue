#ifndef OPENTISSUE_UTILITY_UTILITY_NUMERIC_CAST_H
#define OPENTISSUE_UTILITY_UTILITY_NUMERIC_CAST_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <limits>
#include <stdexcept>
#include <string>

namespace OpenTissue {
namespace utility {

template<typename destination_type, typename source_type>
inline destination_type numeric_cast(source_type const value)
{
#ifndef NDEBUG
    using destination_limit = std::numeric_limits<destination_type>;
    using source_limit = std::numeric_limits<source_type>;

    bool const positive_overflow_possible = destination_limit::max() < source_limit::max();
    bool const negative_overflow_possible = source_limit::is_signed || (destination_limit::lowest() > source_limit::lowest());

    // unsigned <-- unsigned
    if(!destination_limit::is_signed && !source_limit::is_signed)
    {
        if(positive_overflow_possible && (value > destination_limit::max()))
        {
            throw std::overflow_error(__PRETTY_FUNCTION__ +
                                      std::string(": positive overflow"));
        }
    }
    // unsigned <-- signed
    else if(!destination_limit::is_signed && source_limit::is_signed)
    {
        if(positive_overflow_possible && (value > destination_limit::max()))
        {
            throw std::overflow_error(__PRETTY_FUNCTION__ +
                                      std::string(": positive overflow"));
        }
        else if(negative_overflow_possible && (value < 0))
        {
            throw std::overflow_error(__PRETTY_FUNCTION__ +
                                      std::string(": negative overflow"));
        }

    }
    // signed <-- unsigned
    else if(destination_limit::is_signed && !source_limit::is_signed)
    {
        if(positive_overflow_possible && (value > destination_limit::max()))
        {
            throw std::overflow_error(__PRETTY_FUNCTION__ +
                                      std::string(": positive overflow"));
        }
    }
    // signed <-- signed
    else if(destination_limit::is_signed && source_limit::is_signed)
    {
        if(positive_overflow_possible && (value > destination_limit::max()))
        {
            throw std::overflow_error(__PRETTY_FUNCTION__ +
                                      std::string(": positive overflow"));
        }
        else if(negative_overflow_possible && (value < destination_limit::lowest()))
        {
            throw std::overflow_error(__PRETTY_FUNCTION__ +
                                      std::string(": negative overflow"));
        }
    }
#endif

    // limits have been checked, therefore safe to cast
    return static_cast<destination_type>(value);
}

} // namespace utility
} // namespace OpenTissue

//OPENTISSUE_UTILITY_UTILITY_NUMERIC_CAST_H
#endif
