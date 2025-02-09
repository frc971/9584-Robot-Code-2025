#pragma once

#include "Constants.h"
#include <networktables/NetworkTable.h>

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class NetworkTables {
    public:
        NetworkTables();
        double ControllerVelocityCurveExponent();
    

    private:
        std::shared_ptr<nt::NetworkTable> table;
        const std::string kTableName = "Tuning Constants";
        const std::string kControllerVelocityCurveExponentName = "controllerVelocityCurveExponent";
        
};