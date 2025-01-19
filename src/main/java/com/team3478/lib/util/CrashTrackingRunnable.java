package com.team3478.lib.util;

///////////////////////////////////////////////////////////////////////////////
// Description: Runnable class with reports all uncaught throws logger object.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public abstract class CrashTrackingRunnable implements Runnable {
  @Override
  public final void run() {
    try {
      runCrashTracked();
    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  public abstract void runCrashTracked();
}
