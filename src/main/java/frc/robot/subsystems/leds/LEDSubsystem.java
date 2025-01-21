// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  public enum LEDState {
    RAINBOW,
    ALLIANCE,
    TEAM,
    HAS_NOTE,
    NO_TARGET,
    TARGET_LOCKED  
  }

  private LEDState m_state = LEDState.RAINBOW;

  private final AddressableLED m_led = new AddressableLED(Constants.LEDS.PWMPort);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDS.Length);
  private final Distance kLedSpacing = Meters.of(1 / 30.0);

  private final LinearVelocity kScrollingSpeed = MetersPerSecond.of(0.5);

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final LEDPattern m_redAlliance = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kDarkRed);
  private final LEDPattern m_scrollingRedAlliance = m_redAlliance.scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final LEDPattern m_blueAlliance = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kDarkBlue);
  private final LEDPattern m_scrollingBlueAlliance = m_blueAlliance.scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
  private final LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(kScrollingSpeed, kLedSpacing);

  private final LEDPattern m_team = LEDPattern.solid(new Color(0, 200, 50));
  private final LEDPattern m_scrollTeam = m_team.mask(mask);

  private final LEDPattern m_note = LEDPattern.solid(new Color(255, 43, 0));
  private final LEDPattern m_scrollNote = m_note.mask(mask);

  private final LEDPattern m_noTarget = LEDPattern.solid(Color.kRed);
  private final LEDPattern m_breatheNoTarget = m_noTarget.breathe(Seconds.of(1));

  private final LEDPattern m_target = LEDPattern.solid(Color.kGreen);
  private final LEDPattern m_breatheTarget = m_target.breathe(Seconds.of(0.5));

  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLEDState(LEDState state) {
    m_state = state;
  }

  @Override
  public void periodic() {
    switch (m_state) {
      case RAINBOW:
        m_scrollingRainbow.applyTo(m_ledBuffer);
        break;
      case ALLIANCE:
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          m_scrollingRedAlliance.applyTo(m_ledBuffer);
        } else {
          m_scrollingBlueAlliance.applyTo(m_ledBuffer);
        }
        break;
      case TEAM:
        m_scrollTeam.applyTo(m_ledBuffer);
        break;
      case HAS_NOTE:
        m_scrollNote.applyTo(m_ledBuffer);
        break;
      case NO_TARGET:
        m_breatheNoTarget.applyTo(m_ledBuffer);
        break;
      case TARGET_LOCKED:
        m_breatheTarget.applyTo(m_ledBuffer);
        break;
      default:
        m_scrollingRainbow.applyTo(m_ledBuffer);
        break;
    }

    // DriverStation State Overrides
    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        m_scrollingRedAlliance.applyTo(m_ledBuffer);
      } else {
        m_scrollingBlueAlliance.applyTo(m_ledBuffer);
      }
    } else if (DriverStation.isTest()) {
      m_scrollingRainbow.applyTo(m_ledBuffer);
    }

    m_led.setData(m_ledBuffer);
  }
}