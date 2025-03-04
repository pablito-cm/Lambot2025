package com.team3478.frc2025.auto.actions;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  * Action Interface, an interface that describes an iterative action. It is run by an autonomous
// action, called by the
//  * method runAction in AutoModeBase (or more commonly in autonomous modes that extend
// AutoModeBase)
//  *
//  * @see com.team3478.frc2023.auto.modes.AutoModeBase#runAction
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

public interface Action {
  /** Run code once when the action is started, for setup */
  void start();

  /**
   * Called by runAction in AutoModeBase iteratively until isFinished returns true. Iterative logic
   * lives in this method
   */
  void update();

  /**
   * Returns whether or not the code has finished execution. When implementing this interface, this
   * method is used by the runAction method every cycle to know when to stop running the action
   *
   * @return boolean
   */
  boolean isFinished();

  /** Run code once when the action finishes, usually for clean up */
  void done();
}
