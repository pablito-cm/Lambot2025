package com.team3478.lib.swerve;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  * Class to represents the point in a path.
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

public class SwerveSetpoint {
  public ChassisSpeeds mChassisSpeeds;
  public SwerveModuleState[] mModuleStates;

  public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
    this.mChassisSpeeds = chassisSpeeds;
    this.mModuleStates = initialStates;
  }

  @Override
  public String toString() {
    String ret = mChassisSpeeds.toString() + "\n";
    for (int i = 0; i < mModuleStates.length; ++i) {
      ret += "  " + mModuleStates[i].toString() + "\n";
    }
    return ret;
  }
}
