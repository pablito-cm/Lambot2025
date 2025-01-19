package com.team3478.lib.spline;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  *
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

public abstract class Spline1d {
  public abstract double getPosition(double t);

  // ds/dt
  public abstract double getVelocity(double t);

  // ds^2/dt
  public abstract double getAcceleration(double t);

  // ds^3/dt
  public abstract double getJerk(double t);
}
