package com.team3478.frc2025.auto.actions;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  * Action to wait for a given amount of time To use this Action, call runAction(new
// WaitAction(your_time))
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {
  private final double mTimeToWait;
  private double mStartTime;

  public WaitAction(double timeToWait) {
    mTimeToWait = timeToWait;
  }

  @Override
  public void start() {
    mStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {}

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait;
  }

  @Override
  public void done() {}
}
