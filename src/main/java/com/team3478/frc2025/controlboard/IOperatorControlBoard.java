package com.team3478.frc2025.controlboard;

///////////////////////////////////////////////////////////////////////////////
// Description: Interface con las funciones para leer los botones y sticks del control del operador.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public interface IOperatorControlBoard {
  double getLeftStickX();

  double getLeftStickY();

  double getRightStickX();

  double getRightStickY();

  double getTriggersDif();

  int getDPad();

  boolean getYButton();

  boolean getXButton();

  boolean getBButton();

  boolean getAButton();

  boolean getRightBumper();

  boolean getLeftBumper();

  boolean getStartButton();

  boolean getSelectButton();
}
