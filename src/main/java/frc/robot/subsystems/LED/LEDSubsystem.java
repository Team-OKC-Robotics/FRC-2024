package frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(4);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LED.Length);
    private int m_rainbowFirstPixelHue;
    
public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
}

@Override
public void periodic() {}

public void rainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
}

public void stopLED() {
    m_led.stop();
}

private void setFrontAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
        m_ledBuffer.setLED(i, color);
    }
}

public void setFrontHalf() {
    for (int i = 0; i < m_ledBuffer.getLength() / 2; i++) {
        if (i < m_ledBuffer.getLength() / 2) {
            m_ledBuffer.setLED(i, Color.kBlue);

        } else {
            m_ledBuffer.setLED(i, Color.kRed);
        }
    }
}

public void setBackAll(Color color) {
    for (var i = m_ledBuffer.getLength() / 2; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, color);
    }
    
}

public void setAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
}
}
