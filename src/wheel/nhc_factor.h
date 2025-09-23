/*
 * Wheel module: NHC factor (currently identical to NhcFactor in factors/).
 * Implemented as a thin subclass to keep wheel code paths decoupled.
 */
#ifndef WHEEL_NHC_FACTOR_H
#define WHEEL_NHC_FACTOR_H

#include "src/factors/nhc_factor.h"

class WheelNhcFactor : public NhcFactor {
public:
    using NhcFactor::NhcFactor; // inherit constructors
};

#endif // WHEEL_NHC_FACTOR_H

