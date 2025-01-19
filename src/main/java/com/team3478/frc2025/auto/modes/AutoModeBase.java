package com.team3478.frc2025.auto.modes;

///////////////////////////////////////////////////////////////////////////////
// Description: An abstract class that is the basis of the robot's autonomous routines. This is
// implemented in auto modes (which are routines that do actions).
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.auto.AutoModeEndedException;
import com.team3478.frc2025.auto.actions.Action;
import com.team3478.frc2025.auto.actions.NoopAction;
import edu.wpi.first.wpilibj.DriverStation;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public abstract class AutoModeBase {
  protected final double mUpdateRate = 1.0 / 50.0;
  protected boolean mActive = false;
  protected boolean mIsInterrupted = false;

  protected abstract void routine() throws AutoModeEndedException;

  public void run() {
    mActive = true;

    try {
      routine();
    } catch (AutoModeEndedException e) {
      DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
      return;
    }

    done();
  }

  public void done() {
    LoggerManager.log(LogLevel.INFO, "Auto mode done");
  }

  public void stop() {
    mActive = false;
  }

  public boolean isActive() {
    return mActive;
  }

  public boolean isActiveWithThrow() throws AutoModeEndedException {
    if (!isActive()) {
      throw new AutoModeEndedException();
    }

    return isActive();
  }

  public void waitForDriverConfirm() throws AutoModeEndedException {
    if (!mIsInterrupted) {
      interrupt();
    }
    runAction(new NoopAction());
  }

  public void interrupt() {
    LoggerManager.log(LogLevel.INFO, "** Auto mode interrrupted!");
    mIsInterrupted = true;
  }

  public void resume() {
    LoggerManager.log(LogLevel.INFO, "** Auto mode resumed!");
    mIsInterrupted = false;
  }

  public void runAction(Action action) throws AutoModeEndedException {
    isActiveWithThrow();
    long waitTime = (long) (mUpdateRate * 1000.0);

    // Wait for interrupt state to clear
    while (isActiveWithThrow() && mIsInterrupted) {
      try {
        Thread.sleep(waitTime);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    action.start();

    // Run action, stop action on interrupt, non active mode, or done
    while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
      action.update();

      try {
        Thread.sleep(waitTime);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    action.done();
  }

  public boolean getIsInterrupted() {
    return mIsInterrupted;
  }
}
