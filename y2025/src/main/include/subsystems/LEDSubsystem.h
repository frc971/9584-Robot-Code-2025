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
    m_animationCounter += 0.02;
  }

  // LED pattern methods
  void SetSolidColor(const frc::Color& color) {
    m_currentPattern = Pattern::Solid;
    m_primaryColor = color;
  }

  void SetGradientWave(const frc::Color& targetColor) {
    m_currentPattern = Pattern::GradientWave;
    m_primaryColor = targetColor;
  }

  void SetAllianceGradient(const frc::Color& fromColor,
                           const frc::Color& toColor) {
    m_currentPattern = Pattern::AllianceGradient;
    m_primaryColor = fromColor;
    m_secondaryColor = toColor;
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
  void SetDisabledMode() { SetRainbow(); };

  void SetTeleopMode() {
    auto alliance = frc::DriverStation : GetAlliance();
    if (alliance) {
      if (alliance.value() == frc::DriverStation::Alliance::kRed) {
        SetAllianceGradient(frc::Color::kGreen, kSpartanGold);
      } else {
        SetAllianceGradient(frc::Color::kBlue, kSpartanGold);
      }
    } else {
      SetGradientWave(kSpartanGold);
    }
  }

  void SetAutonomousMode() { SetChase(kSpartanGold); }

  void SetAllianceColor() {
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance) {
      if (alliance.value() ==
          frc::DriverStation::Alliance::kRed) {  // leds r bugged so red is
                                                 // green
        SetSolidColor(frc::Color::kGreen);
      } else {
        SetSolidColor(frc::Color::kBlue);
      }
    } else {
      SetSolidColor(frc::Color::kWhite);
    }
  }

 private:
  static constexpr int kPWMPort = 1;  // where the yellow wire is
  static constexpr int kLength =
      30;  // 30 leds -- need to change to the amt on the robot

  static constexpr frc::Color kSpartanGold{0xF1 / 255.0, 0xC3 / 255.0,
                                           0x31 / 255.0};

  frc::AddressableLED m_led{kPWMPort};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

  enum class Pattern {
    Solid,
    Rainbow,
    Chase,
    Blink,
    Alternating,
    Breathe,
    GradientWave,
    AllianceGradient,
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
      case Pattern::GradientWave:
        UpdateGradientWave();
        break;
      case Pattern::AllianceGradient:
        UpdateAllianceGradient();
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
    int rainbowFirstPixelHue = static_cast<int>(m_animationCounter * 180) % 180;

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

  void UpdateGradientWave() {
    double offset = std::fmod(m_animationCounter * 0.5, 1.0);

    for (int i = 0; i < kLength; i++) {
      double position =
          std::fmod(static_cast<double>(i) / kLength + offset, 1.0);

      int r = static_cast<int>(m_primaryColor.red * 255 * position);
      int g = static_cast<int>(m_primaryColor.green * 255 * position);
      int b = static_cast<int>(m_primaryColor.blue * 255 * position);
      m_ledBuffer[i].SetRGB(r, g, b);
    }
  }

  void UpdateAllianceGradient() {
    double offset = std::fmod(m_animationCounter * 0.5, 1.0);

    for (int i = 0; i < kLength; i++) {
      double position =
          std::fmod(static_cast<double>(i) / kLength + offset, 1.0);

      int r = static_cast<int>(
          (m_primaryColor.red +
           (m_secondaryColor.red - m_primaryColor.red) * position) *
          255);
      int g = static_cast<int>(
          (m_primaryColor.green +
           (m_secondaryColor.green - m_primaryColor.green) * position) *
          255);
      int b = static_cast<int>(
          (m_primaryColor.blue +
           (m_secondaryColor.blue - m_primaryColor.blue) * position) *
          255);
      m_ledBuffer[i].SetRGB(r, g, b);
    }
  }
};