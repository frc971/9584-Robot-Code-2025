#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "units/base.h"
namespace DriveConstants {
    constexpr auto slewTranslateLimit = units::scalar_t(0.6) / 1_s;
    constexpr auto slewRotateLimit = units::scalar_t(1) / 1_s;
}


#endif // CONSTANTS_H