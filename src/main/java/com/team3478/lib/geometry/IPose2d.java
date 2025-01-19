package com.team3478.lib.geometry;

///////////////////////////////////////////////////////////////////////////////
// Description: -
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
  Pose2d getPose();

  S transformBy(Pose2d transform);

  S mirror();
}
