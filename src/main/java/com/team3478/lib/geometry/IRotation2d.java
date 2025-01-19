package com.team3478.lib.geometry;

///////////////////////////////////////////////////////////////////////////////
// Description: -
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

public interface IRotation2d<S> extends State<S> {
  Rotation2d getRotation();

  S rotateBy(Rotation2d other);

  S mirror();
}
