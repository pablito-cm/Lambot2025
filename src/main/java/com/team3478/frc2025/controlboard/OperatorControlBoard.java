package com.team3478.frc2025.controlboard;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con las funciones para leer los botones y sticks del control del operador.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.Constants;

public class OperatorControlBoard implements IOperatorControlBoard {

  private static OperatorControlBoard mInstance = null;

  public static OperatorControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorControlBoard();
    }
    return mInstance;
  }

  private final XboxController mController;

  private OperatorControlBoard() {
    mController = new XboxController(Constants.ControlBoard.kOperatorControlPort);
  }

  @Override
  public double getLeftStickX() {
    return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
  }

  @Override
  public double getLeftStickY() {
    return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
  }

  @Override
  public double getRightStickX() {
    return mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
  }

  @Override
  public double getRightStickY() {
    return mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
  }

  @Override
  public double getTriggersDif() {
    return (mController.getTrigger(XboxController.Side.RIGHT)
        - mController.getTrigger(XboxController.Side.LEFT));
  }

  @Override
  public int getDPad() {
    return mController.getDPad();
  }

  @Override
  public boolean getYButton() {
    return mController.getButtonReleased(XboxController.Button.Y);
  }

  @Override
  public boolean getXButton() {
    return mController.getButtonReleased(XboxController.Button.X);
  }

  @Override
  public boolean getBButton() {
    return mController.getButtonReleased(XboxController.Button.B);
  }

  @Override
  public boolean getAButton() {
    return mController.getButtonReleased(XboxController.Button.A);
  }

  @Override
  public boolean getRightBumper() {
    return mController.getButton(XboxController.Button.RB);
  }

  @Override
  public boolean getLeftBumper() {
    return mController.getButton(XboxController.Button.LB);
  }

  @Override
  public boolean getStartButton() {
    return mController.getButtonReleased(XboxController.Button.START);
  }

  @Override
  public boolean getSelectButton() {
    return mController.getButtonReleased(XboxController.Button.BACK);
  }
}
