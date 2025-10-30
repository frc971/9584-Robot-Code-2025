// LEDController.cpp
#include <frc/AddressableLED.h>
#include <frc/Timer.h>
#include <array>
#include <cmath>
#include <cstdint>

class LEDController {
 public:
  static constexpr int kLEDPWMPort = 0;  // RoboRIO PWM port for LEDs
  static constexpr int kNumLEDs = 60;    // Number of LEDs on strip

  LEDController() {
    m_led.SetLength(kNumLEDs);
    m_led.SetColorOrder(frc::AddressableLED::ColorOrder::kGRB);  // AndyMark strip = GRB
    for (auto &led : m_ledBuffer)
      led.SetRGB(0, 0, 0);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
  }

  void Periodic() {
    m_led.SetData(m_ledBuffer);
  }

  void SetSolidColor(uint8_t r, uint8_t g, uint8_t b) {
    for (auto &led : m_ledBuffer)
      led.SetRGB(r, g, b);
  }

  void SetRainbow() {
    static double startHue = 0.0;
    startHue += 1.0;
    if (startHue >= 180.0) startHue -= 180.0;

    for (int i = 0; i < kNumLEDs; ++i) {
      double hue = std::fmod(startHue + (i * (180.0 / kNumLEDs)), 180.0);
      m_ledBuffer[i].SetHSV(static_cast<int>(hue), 255, 128);
    }
  }

  void SetFlashing(uint8_t r, uint8_t g, uint8_t b, double frequencyHz = 2.0) {
    double time = frc::Timer::GetFPGATimestamp().value();
    bool on = std::fmod(time * frequencyHz, 1.0) < 0.5;
    for (auto &led : m_ledBuffer)
      led.SetRGB(on ? r : 0, on ? g : 0, on ? b : 0);
  }

  void SetForRobotState(bool isEnabled, bool isAutonomous) {
    if (!isEnabled)
      SetSolidColor(255, 0, 0);   // Red = disabled
    else if (isAutonomous)
      SetRainbow();               // Rainbow = autonomous
    else
      SetSolidColor(0, 0, 255);   // Blue = teleop
  }

 private:
  frc::AddressableLED m_led{kLEDPWMPort};
  std::array<frc::AddressableLED::LEDData, kNumLEDs> m_ledBuffer;
};
