// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_led = new AddressableLED(Constants.LEDS.PWMPort);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDS.Length);
  private int m_rainbowFirstPixelHue;
  private int blinkNum;

  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 12.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip,
  // moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), kLedSpacing);

  /** Creates a new Leds. */
  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void rainbow() {
    // Update the buffer with the rainbow animation
    m_scrollingRainbow.applyTo(m_ledBuffer);
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }

  public void setAll(Color color) {
    blinkNum = (blinkNum + 1) % 50;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
      if (blinkNum / 5 == i % 10) {
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
    m_led.setData(m_ledBuffer);

  }

  public void setLEDs() {
    rainbow();
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {

  }
}