#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class Climber : public frc2::SubsystemBase {
    public:
     Climber();

     frc2::CommandPtr Climb();
     frc2::CommandPtr Unclimb();

    private:
     ctre::phoenix6::hardware::TalonFX m_motor{13, "rio"};
     ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};
     const units::angle::turn_t CLIMB_DISTANCE = 1_tr;
};