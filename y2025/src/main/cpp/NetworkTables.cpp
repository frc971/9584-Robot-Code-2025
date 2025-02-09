#include "NetworkTables.h"
#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

NetworkTables::NetworkTables() {
    auto inst = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable(kTableName);
    table ->SetDefaultNumber(kControllerVelocityCurveExponentName, DriveConstants::kControllerVelocityCurveExponent);
    table ->SetPersistent(kControllerVelocityCurveExponentName);
}

double NetworkTables::ControllerVelocityCurveExponent() {
    return table->GetNumber(kControllerVelocityCurveExponentName, DriveConstants::kControllerVelocityCurveExponent);
}