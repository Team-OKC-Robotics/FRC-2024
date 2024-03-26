package frc.robot.utils;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class POVButton extends Trigger {
  /**
   * Creates a trigger button for triggering commands.
   *
   * @param joystick The GenericHID object that has the trigger (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The axis number (see {@link GenericHID#getRawAxis(int) }
   * @param threshold How far the trigger must be pressed before activated (0.0 to 1.0)
   */
  public POVButton(GenericHID joystick, int axisNumber) {
    super(() -> joystick.getPOV(axisNumber) >= 0);
    requireNonNullParam(joystick, "joystick", "TriggerButton");
  }
}
