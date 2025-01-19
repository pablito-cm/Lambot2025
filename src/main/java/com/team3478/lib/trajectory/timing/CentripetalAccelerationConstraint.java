package com.team3478.lib.trajectory.timing;

// ///////////////////////////////////////////////////////////////////////////////
// // Description:
// //  *
// // Authors: -
// // Notes:
// //  - Obtenido de: https://github.com/Team254/FRC-2023-Public
// ///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Pose2dWithMotion;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2dWithMotion> {
  final double mMaxCentripetalAccel;

  public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
    mMaxCentripetalAccel = max_centripetal_accel;
  }

  @Override
  public double getMaxVelocity(final Pose2dWithMotion state) {
    return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
  }

  @Override
  public MinMaxAcceleration getMinMaxAcceleration(
      final Pose2dWithMotion state, final double velocity) {
    return MinMaxAcceleration.kNoLimits;
  }
}
