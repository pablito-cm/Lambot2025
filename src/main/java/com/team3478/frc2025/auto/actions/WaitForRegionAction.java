package com.team3478.frc2025.auto.actions;

///////////////////////////////////////////////////////////////////////////////
// Description:
// * Accion para esperar que se llege a una region
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.subsystems.Drive;
import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Translation2d;

public class WaitForRegionAction implements Action {
  private Translation2d mBottomLeft;
  private Translation2d mTopRight;
  private Drive mDrive = null;

  public WaitForRegionAction(Translation2d bottomLeft, Translation2d topRight) {
    mBottomLeft = bottomLeft;
    mTopRight = topRight;
  }

  @Override
  public void start() {
    mDrive = Drive.getInstance();
  }

  @Override
  public void update() {}

  @Override
  public boolean isFinished() {
    Pose2d robotState = mDrive.getOdometry().getEstimatedPosition();
    return robotState.getTranslation().x() < mTopRight.x()
        && robotState.getTranslation().x() > mBottomLeft.x()
        && robotState.getTranslation().y() < mBottomLeft.y()
        && robotState.getTranslation().y() > mTopRight.y();
  }

  @Override
  public void done() {}
}
