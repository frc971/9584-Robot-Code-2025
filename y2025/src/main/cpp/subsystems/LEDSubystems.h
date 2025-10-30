// subsystems/LEDController.h
#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <array>
#include <cmath>

class LEDController : public frc2::SubsystemBase {
 public:
  static constexpr int kLEDPWMPort = 0;  // your LED PWM port
  static constexpr int kNumLEDs = 60;    // number of LEDs on your strip

  LEDController();

  void Periodic() override;

  void SetSolidColor(uint8_t r, uint8_t g, uint8_t b);
  void SetRainbow();
  void SetFlashing(uint8_t r, uint8_t g, uint8_t b, double frequencyHz = 2.0);
  void SetForRobotState(bool isEnabled, bool isAutonomous);

 private:
  frc::AddressableLED m_led{kLEDPWMPort};
  std::array<frc::AddressableLED::LEDData, kNumLEDs> m_ledBuffer;
};
