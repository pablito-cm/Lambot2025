package com.team3478.frc2025.controlboard;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para juntar las clases de los controlles
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public class ControlBoard {
  // Objeto para hacer el singleton
  private static ControlBoard mInstance = null;

  // Funcion para leer el singleton object
  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }
    return mInstance;
  }

  private IDriverControlBoard mDriverControlBoard;
  private IOperatorControlBoard mOperatorControlBoard;

  private ControlBoard() {
    mDriverControlBoard = DriverControlBoard.getInstance();
    mOperatorControlBoard = OperatorControlBoard.getInstance();
  }

  public double driveXVelocity() {
    return mDriverControlBoard.getLeftStickX();
  }

  public double driveYVelocity() {
    return mDriverControlBoard.getLeftStickY();
  }

  public double driveTurnVelocity() {
    return mDriverControlBoard.getRightStickX();
  }

  public boolean ampAlignAction() {
    return mDriverControlBoard.getAButton();
  }

  public boolean down() {
    return mDriverControlBoard.getXButton();
  }

  public boolean up() {
    return mDriverControlBoard.getAButton();
  }

  public boolean activate() {
    return mDriverControlBoard.getBButton();
  }

  public double intakeEat() {
    return mDriverControlBoard.getTriggersDif();
  }

  public double elevatorManual() {
    return mOperatorControlBoard.getLeftStickY();
  }

  public double shooterManual() {
    return mOperatorControlBoard.getRightStickY();
  }

  public boolean ampPositionAction() {
    return mOperatorControlBoard.getAButton();
  }

  public boolean feederPositionAction() {
    return mOperatorControlBoard.getXButton();
  }

  public boolean safePositionAction() {
    return mOperatorControlBoard.getYButton();
  }

  public boolean toogleShooterState() {
    return mOperatorControlBoard.getBButton();
  }

  public boolean shootAction() {
    return mOperatorControlBoard.getTriggersDif() > 0.5;
  }

  public boolean shooterEatAction() {
    return mOperatorControlBoard.getTriggersDif() < -0.5;
  }

  public boolean operatorElevatorMode() {
    return mOperatorControlBoard.getStartButton(); // && mOperatorControlBoard.getSelectButton()
  }

  public boolean slowShootAction() {
    return mOperatorControlBoard.getRightBumper();
  }

  public boolean trapPosition1Action() {
    return mOperatorControlBoard.getAButton();
  }

  public boolean trapPosition2Action() {
    return mOperatorControlBoard.getBButton();
  }

  public boolean trapPosition3Action() {
    return mOperatorControlBoard.getXButton();
  }

  public int escalatorPosition() {
    int dpadTolerance = 5;
    int dpad = mOperatorControlBoard.getDPad();
    if ((dpad >= 0 && dpad < dpadTolerance) || dpad > (360 - dpadTolerance)) {
      return 1;
    } else if (dpad < (90 + dpadTolerance) && dpad > (90 - dpadTolerance)) {
      return 2;
    } else if (dpad < (180 + dpadTolerance) && dpad > (180 - dpadTolerance)) {
      return 3;
    } else if (dpad < (270 + dpadTolerance) && dpad > (270 - dpadTolerance)) {
      return 4;
    }
    return 0;
  }

  public boolean manualEscalatorUp() {
    return mOperatorControlBoard.getRightBumper();
  }

  public boolean manualEscalatorDown() {
    return mOperatorControlBoard.getLeftBumper();
  }
}
