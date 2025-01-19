package com.team3478.frc2025.auto.modes;

///////////////////////////////////////////////////////////////////////////////
// Description: Archivo con el autonomo para hacer nada.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.auto.AutoModeEndedException;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public class DoNothingAutoMode extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    LoggerManager.log(LogLevel.INFO, "Do nothing auto mode");
  }
}
