// subsystems/LEDController.cpp
#include <frc/Timer.h>

#include "subsystems/LEDController.h"

LEDController::LEDController() {
  m_led.SetLength(kNumLEDs);
  m_led.SetColorOrder(frc::AddressableLED::ColorOrder::kGRB);
  for (auto &led : m_ledBuffer) led.SetRGB(0, 0, 0);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}

void LEDController::Periodic() { m_led.SetData(m_ledBuffer); }

void LEDController::SetSolidColor(uint8_t r, uint8_t g, uint8_t b) {
  for (auto &led : m_ledBuffer) led.SetRGB(r, g, b);
}

void LEDController::SetRainbow() {
  static double startHue = 0.0;
  startHue += 1.0;
  if (startHue >= 180.0) startHue -= 180.0;
  for (int i = 0; i < kNumLEDs; ++i) {
    double hue = std::fmod(startHue + (i * (180.0 / kNumLEDs)), 180.0);
    m_ledBuffer[i].SetHSV(static_cast<int>(hue), 255, 128);
  }
}

void LEDController::SetFlashing(uint8_t r, uint8_t g, uint8_t b, double freq) {
  double time = frc::Timer::GetFPGATimestamp().value();
  bool on = std::fmod(time * freq, 1.0) < 0.5;
  for (auto &led : m_ledBuffer) led.SetRGB(on ? r : 0, on ? g : 0, on ? b : 0);
}

void LEDController::SetForRobotState(bool isEnabled, bool isAutonomous) {
  if (!isEnabled)
    SetSolidColor(255, 0, 0);  // red = disabled
  else if (isAutonomous)
    SetRainbow();  // rainbow = auto
  else
    SetSolidColor(0, 0, 255);  // blue = teleop
}
