package com.team3478.lib.util;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para hacer los calculos de la fisca de un motor.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public class MotorUtil {

  private double gearReduction = 1;

  public MotorUtil(double _reduction) {
    gearReduction = _reduction;
  }

  public double GetMaxVelocity(double wheelradius, int motorMaxRPM) {
    return GetMaxAngularSpeed(motorMaxRPM) * wheelradius;
  }

  private double GetMaxAngularSpeed(int motorMaxRPM) {
    return (((2 * Math.PI) * motorMaxRPM) / (double) 60) * gearReduction; // w=(2pi)(rpm)/60
  }

  public double GetPercentageFromVelocity(double velocity, double wheelRadius, int motorMaxRPM) {
    double percentaje = velocity / GetMaxVelocity(wheelRadius, motorMaxRPM);
    // Seguridad
    if (percentaje > 1) {
      percentaje = 1;
    } else if (percentaje < -1) {
      percentaje = -1;
    }
    return percentaje;
  }
}
