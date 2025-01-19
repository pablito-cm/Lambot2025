package com.team3478.frc2025.controlboard;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para realizar el mapping de un control de xbox
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.util.Util;
import edu.wpi.first.wpilibj.Joystick;

public class XboxController {
  private final Joystick mController;

  public enum Side {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }

  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    BACK(7),
    START(8),
    L_JOYSTICK(9),
    R_JOYSTICK(10);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }

  XboxController(int port) {
    mController = new Joystick(port);
  }

  double getJoystick(Side side, Axis axis) {
    // Esto lo dejamos en 0 para poder aplicar esto
    // en cada lugar que usa los sticks con diferentes rangos
    double deadband = 0;

    boolean left = side == Side.LEFT;
    boolean y = axis == Axis.Y;
    // multiplies by -1 if y-axis (inverted normally)
    return Util.handleDeadband(
        (y ? -1 : 1) * mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0)), deadband);
  }

  double getTrigger(Side side) {
    // Esto lo dejamos en 0 para poder aplicar esto
    // en cada lugar que usa los sticks con diferentes rangos
    double deadband = 0;
    return Util.handleDeadband(mController.getRawAxis(side == Side.LEFT ? 2 : 3), deadband);
  }

  boolean getButton(Button button) {
    return mController.getRawButton(button.id);
  }

  boolean getButtonReleased(Button button) {
    return mController.getRawButtonReleased(button.id);
  }

  int getDPad() {
    return mController.getPOV();
  }

  public boolean getButtonPressed(Button y) {
    return false;
  }
}
