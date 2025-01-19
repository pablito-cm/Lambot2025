package com.team3478.frc2025.auto;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para crear los selectores de autonomos en la driver station y cargar el
// autonomo a correr.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.auto.modes.*;
import com.team3478.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public class AutoModeSelector {

  // AutoModes
  enum DesiredMode {
    DO_NOTHING
  }

  private DesiredMode mCachedDesiredMode = null;
  private SendableChooser<DesiredMode> mModeChooser;
  private Optional<AutoModeBase> mAutoMode = Optional.empty();

  public AutoModeSelector() {
    // Create smartdashboard auto selector
    mModeChooser = new SendableChooser<>();
    mModeChooser.setDefaultOption("DO_NOTHING", DesiredMode.DO_NOTHING);
    SmartDashboard.putData("Auto mode", mModeChooser);
  }

  public void updateModeCreator() {
    DesiredMode desiredMode = mModeChooser.getSelected();
    if (desiredMode == null) {
      desiredMode = DesiredMode.DO_NOTHING;
    }
    if (mCachedDesiredMode != desiredMode) {
      LoggerManager.log(
          LogLevel.INFO,
          "Auto selection changed, updating creator: desiredMode->"
              + desiredMode.name()
              + ", Is Red Allience->"
              + Util.isRedAllience());
      mAutoMode = getAutoModeForParams(desiredMode);
    }
    mCachedDesiredMode = desiredMode;
  }

  private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
    switch (mode) {
      case DO_NOTHING:
        return Optional.of(new DoNothingAutoMode());
      default:
        break;
    }
    LoggerManager.log(LogLevel.ERROR, "No valid auto mode found for: " + mode);
    return Optional.empty();
  }

  public void reset() {
    mAutoMode = Optional.empty();
    mCachedDesiredMode = null;
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
  }

  public Optional<AutoModeBase> getAutoMode() {
    return mAutoMode;
  }
}
