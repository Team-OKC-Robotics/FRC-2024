// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

 
  private final AddressableLED m_led = new AddressableLED(Constants.LEDS.PWMPort);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDS.Length);
  private int m_rainbowFirstPixelHue;
  private int blinkNum;


  /** Creates a new Leds. */
  public LEDSubsystem() {
     m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

 public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
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