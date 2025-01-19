///////////////////////////////////////////////////////////////////////////////
// Description: Archivo donde se declararan todas las constantes configurables del programa.
// Authors: -
// Notes:
//  - Usar clases estaticas para agrupar las variables por secciones.
///////////////////////////////////////////////////////////////////////////////

package com.team3478.frc2025;

import com.team3478.lib.swerve.SwerveKinematicLimits;
import lambotlogs.logger.LoggerManager.LogLevel;

public final class Constants {

  // Loop period in seconds
  public static final double kLooperDt = 0.01;
  // Log level to use (para competencia dejar en info)
  public static final LogLevel klogLevel = LogLevel.INFO;
  // Mode to use for metrics (empty string accept all)
  public static final String kmetricMode = "";
  // Mode to use for smartDashboard (emtpy string accept all)
  public static final String ksmartdahboardMode = "TESTING"; // "TESTING"; DRIVER

  // Constants for the controlboard
  public static class ControlBoard {
    // Driver controller port
    public static final int kDriverControlPort = 0;
    // Operator controller port
    public static final int kOperatorControlPort = 1;
  }

  // Constants for the drive subsystem
  public static class Drive {
    // Motors current limits
    public static final int kSpeedMotorCurrentLimit = 60;
    public static final int kSteeringMotorCurrentLimit = 30;
    // Variables para offsetear el 0 de las llantas del swerve(steering angle encoders)
    public static final double kFrontRightEncoderInitPos = -47.6367; // -55.6347; // -58.2714;
    public static final double kFrontLeftEncoderInitPos = -46.5; // -131.0449; // -130.9570;
    public static final double kBackRightEncoderInitPos = -127; // -43.3300;
    public static final double kBackLeftEncoderInitPos = -57.3; // -49.2187; // -343.1250;
    // Variables con constantes de las ruedas
    public static final double wheelRadius = 0.0508;
    public static final double wheelTrack = 0.5534;
    // Variables con constantes de los motores
    public static final int kMotorRPM = 6000;
    // Variables con los ids de los cancoders
    public static final int kfrontRightCANCoderID = 9;
    public static final int kfrontLeftCANCoderID = 12;
    public static final int kbackRightCANCoderID = 10;
    public static final int kbackLeftCANCoderID = 11;
    // Variables con los ids de los motores
    public static final int kFrontRightSpeedMotorID = 2;
    public static final int kFrontRightSteeringMotorID = 1;
    public static final int kFrontLeftSpeedMotorID = 8;
    public static final int kFrontLeftSteeringMotorID = 7;
    public static final int kBackRightSpeedMotorID = 4;
    public static final int kBackRightSteeringMotorID = 3;
    public static final int kBackLeftSpeedMotorID = 6;
    public static final int kBackLeftSteeringMotorID = 5;
    // Variable para indicar el numero de modulos de swerve en el robot
    public static int kNumberOfModules = 4;
    // Variables con las posiciones de los moduloes en metros
    // Distancias contra el centro del chassis
    public static final double kXFrontLeftLocation = 0.2767;
    public static final double kYFrontLeftLocation = 0.2767;
    public static final double kXFrontRightLocation = 0.2767;
    public static final double kYFrontRightLocation = -0.2767;
    public static final double kXBackLeftLocation = -0.2767;
    public static final double kYBackLeftLocation = 0.2767;
    public static final double kXBackRightLocation = -0.2767;
    public static final double kYBackRightLocation = -0.2767;
    // Reduccion de los motores de velocidad
    public static final double kGearSpeedReduction = 1 / 6.12;
    public static final double kGearSteeringReduction = 1 / 21.42;
    // Variables para el controlador PD de los motores de steering
    public static final double kPSteeringValue = 0.006;
    public static final double kDSteeringValue = 0.0001;
    // Variable para el acumulador de inercia
    public static final double kDriveTurnDeadband = 0.5;
    public static final double kAcumulatorChange = 0.05;
    public static final double kAcumulatorAlpha = 0.1;
    // Variables para el controlador PD para ajustar el giro del robot
    public static final double kPTurnValueBigError = 0.016;
    public static final double kDTurnValueBigError = 0;
    public static final double kPTurnValueSmallError = 0.02;
    public static final double kDTurnValueSmallError = 0;
    // Variables de seguridad para evitar que se active ir a
    // una posicion de una distancia muy lejana (en metros)
    public static final double kMaxDistanceDifferenceForPath = 4.0;
    // Pure Pursuit Constants
    //// Pure pursuit pid constants
    public static final double kPurePursuitThetakP = 7;
    public static final double kPurePursuitThetakD = 0.0;
    public static final double kPurePursuitPositionkP = 5.0;
    public static final double kPathLookaheadTime = 0.25; // From 1323 (2019)
    public static final double kPathMinLookaheadDistance = 12.0; // From 1323 (2019)
    public static final double kAdaptivePathMinLookaheadDistance = 6.0;
    public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
    public static final double kAdaptiveErrorLookaheadCoefficient = 4.0;
    // Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    public static final double kMaxVelocityMetersPerSecond = 3.5; // Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 8;
    // Restricciones kinematicas del robot **sin limite**
    public static final SwerveKinematicLimits kUncappedKinematicLimits =
        new SwerveKinematicLimits();

    static {
      kUncappedKinematicLimits.kMaxDriveVelocity = Constants.Drive.kMaxVelocityMetersPerSecond;
      kUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
      kUncappedKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
    }

    // REstricciones kinematicas del robot
    public static final SwerveKinematicLimits kSmoothKinematicLimits = new SwerveKinematicLimits();

    static {
      kSmoothKinematicLimits.kMaxDriveVelocity = Constants.Drive.kMaxVelocityMetersPerSecond * .9;
      kSmoothKinematicLimits.kMaxDriveAcceleration =
          Constants.Drive.kMaxAccelerationMetersPerSecondSquared;
      kSmoothKinematicLimits.kMaxSteeringVelocity = 13.09; // radianes/second
    }
  }
}
