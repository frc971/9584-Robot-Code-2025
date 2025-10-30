// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include <array>
#include <cmath>
#include <numbers>

class LEDSubsystem : public frc2::SubsystemBase {
 public:
  LEDSubsystem() {
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
  }

  void Periodic() override {
    UpdateLEDs();
    m_led.SetData(m_ledBuffer);
    m_animationCounter += 0.02;  // 20ms loop time
  }

  // LED pattern methods
  void SetSolidColor(const frc::Color& color) {
    m_currentPattern = Pattern::Solid;
    m_primaryColor = color;
  }

  void SetRainbow() { m_currentPattern = Pattern::Rainbow; }

  void SetChase(const frc::Color& color) {
    m_currentPattern = Pattern::Chase;
    m_primaryColor = color;
  }

  void SetBlink(const frc::Color& color, double frequency = 1.0) {
    m_currentPattern = Pattern::Blink;
    m_primaryColor = color;
    m_blinkFrequency = frequency;
  }

  void SetAlternating(const frc::Color& color1, const frc::Color& color2) {
    m_currentPattern = Pattern::Alternating;
    m_primaryColor = color1;
    m_secondaryColor = color2;
  }

  void SetBreathe(const frc::Color& color) {
    m_currentPattern = Pattern::Breathe;
    m_primaryColor = color;
  }

  void Off() {
    m_currentPattern = Pattern::Off;
    for (auto& led : m_ledBuffer) {
      led.SetRGB(0, 0, 0);
    }
  }

  // Status indicator methods
  void SetDisabledMode() { SetBlink(frc::Color::kOrange, 0.5); }

  void SetTeleopMode() { SetAllianceColor(); }

  void SetAutonomousMode() { SetChase(frc::Color::kGreen); }

  void SetAllianceColor() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance) {
      if (alliance.value() == frc::DriverStation::Alliance::kRed) {
        SetSolidColor(frc::Color::kRed);
      } else {
        SetSolidColor(frc::Color::kBlue);
      }
    } else {
      SetSolidColor(frc::Color::kWhite);
    }
  }

 private:
  static constexpr int kPWMPort = 1;  // Change to your PWM port
  static constexpr int kLength = 30;  // Number of LEDs

  frc::AddressableLED m_led{kPWMPort};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

  enum class Pattern {
    Solid,
    Rainbow,
    Chase,
    Blink,
    Alternating,
    Breathe,
    Off
  };

  Pattern m_currentPattern{Pattern::Off};
  frc::Color m_primaryColor{0, 0, 0};
  frc::Color m_secondaryColor{0, 0, 0};
  double m_animationCounter{0.0};
  double m_blinkFrequency{1.0};

  void UpdateLEDs() {
    switch (m_currentPattern) {
      case Pattern::Solid:
        UpdateSolid();
        break;
      case Pattern::Rainbow:
        UpdateRainbow();
        break;
      case Pattern::Chase:
        UpdateChase();
        break;
      case Pattern::Blink:
        UpdateBlink();
        break;
      case Pattern::Alternating:
        UpdateAlternating();
        break;
      case Pattern::Breathe:
        UpdateBreathe();
        break;
      case Pattern::Off:
        break;
    }
  }

  void UpdateSolid() {
    for (auto& led : m_ledBuffer) {
      led.SetLED(m_primaryColor);
    }
  }

  void UpdateRainbow() {
    int rainbowFirstPixelHue = static_cast<int>(m_animationCounter * 90) % 180;

    for (int i = 0; i < kLength; i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / kLength)) % 180;
      m_ledBuffer[i].SetHSV(hue, 255, 128);
    }
  }

  void UpdateChase() {
    int position = static_cast<int>(m_animationCounter * 20) % kLength;
    int chaseLength = 5;

    for (int i = 0; i < kLength; i++) {
      if (std::abs(i - position) < chaseLength) {
        m_ledBuffer[i].SetLED(m_primaryColor);
      } else {
        m_ledBuffer[i].SetRGB(0, 0, 0);
      }
    }
  }

  void UpdateBlink() {
    double blinkPhase = std::fmod(m_animationCounter * m_blinkFrequency, 1.0);
    bool isOn = blinkPhase < 0.5;

    for (auto& led : m_ledBuffer) {
      if (isOn) {
        led.SetLED(m_primaryColor);
      } else {
        led.SetRGB(0, 0, 0);
      }
    }
  }

  void UpdateAlternating() {
    for (int i = 0; i < kLength; i++) {
      if (i % 2 == 0) {
        m_ledBuffer[i].SetLED(m_primaryColor);
      } else {
        m_ledBuffer[i].SetLED(m_secondaryColor);
      }
    }
  }

  void UpdateBreathe() {
    double brightness = (std::sin(m_animationCounter * 2.0) + 1.0) / 2.0;

    for (auto& led : m_ledBuffer) {
      led.SetRGB(static_cast<int>(m_primaryColor.red * 255 * brightness),
                 static_cast<int>(m_primaryColor.green * 255 * brightness),
                 static_cast<int>(m_primaryColor.blue * 255 * brightness));
    }
  }
};