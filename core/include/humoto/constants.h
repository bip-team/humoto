/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#pragma once


namespace humoto
{
    /// PI constant
    const double g_pi = 3.14159265358979323846;

    /// Gravitational acceleration
    const double g_gravitational_acceleration = 9.8;

    /// Infinity.
    const double g_infinity = 1e20;
    //const double infinity = std::numeric_limits<double>::max();

    /// Generic tolerance, should be used by default.
    const double g_generic_tolerance = 1e-12;
}
