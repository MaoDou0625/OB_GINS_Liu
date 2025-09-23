/*
 * Wheel module: wheel speed factor (currently identical to OdoFactor).
 * Implemented as a thin subclass to keep wheel code paths decoupled.
 */
#ifndef WHEEL_SPEED_FACTOR_H
#define WHEEL_SPEED_FACTOR_H

#include "src/factors/odo_factor.h"

class WheelSpeedFactor : public OdoFactor {
public:
    using OdoFactor::OdoFactor; // inherit constructors
};

#endif // WHEEL_SPEED_FACTOR_H

