package com.team3478.lib.swerve;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase hecha para ajustar la clase de SwerveDrivePoseEstimator a
//              nuestras clases custom.
// Authors: -
// Notes:
//  - Usamos esta en lugar de la de la odometria sencilla ya que nos permite fusionar
//    los valores de los encoder y la camara con un calman filter
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.Constants;
import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lambotlogs.csv.CSVWritable;

public class SwervePoseEstimator implements CSVWritable {

  private edu.wpi.first.math.kinematics.SwerveDriveKinematics KINEMATICS = null;
  private SwerveModulePosition[] modulesPositions = null;
  private SwerveDrivePoseEstimator poseEstimator = null;
  private ChassisSpeeds m_velocity;
  // Fijamos los valores para calibrar el filtro (entre mas bajos mas importancia le da)
  private Matrix<N3, N1> stateStdDevs =
      new Matrix<>(VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)));
  private Matrix<N3, N1> visionMeasurementStdDevs =
      new Matrix<>(VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(10)));

  // Constructor de la clase
  public SwervePoseEstimator(
      double startX, double startY, double startRotation, SwerveModuleState[] states) {
    // Creamos el objeto para calcular la kinematica del swerve (utilizando la libreria de first)
    KINEMATICS =
        new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
            // Front left
            new edu.wpi.first.math.geometry.Translation2d(
                Constants.Drive.kXFrontLeftLocation, Constants.Drive.kYFrontLeftLocation),
            // Front right
            new edu.wpi.first.math.geometry.Translation2d(
                Constants.Drive.kXFrontRightLocation, Constants.Drive.kYFrontRightLocation),
            // Back left
            new edu.wpi.first.math.geometry.Translation2d(
                Constants.Drive.kXBackLeftLocation, Constants.Drive.kYBackLeftLocation),
            // Back right
            new edu.wpi.first.math.geometry.Translation2d(
                Constants.Drive.kXBackRightLocation, Constants.Drive.kYBackRightLocation));
    // Seteamos los estados iniciales de los modulos
    EstimateModulePositions(states);
    // Create pose estimator object
    poseEstimator =
        new SwerveDrivePoseEstimator(
            KINEMATICS,
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(startRotation),
            modulesPositions,
            new edu.wpi.first.math.geometry.Pose2d(
                startX, startY, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(startRotation)),
            stateStdDevs,
            visionMeasurementStdDevs);
    // Inicializamos las chassis speeds en 0
    m_velocity = new ChassisSpeeds();
  }

  // Funcion para calcular los module positions (desde nuestros estados)
  // Considera el cambio en la posicion por la velocidad y el cambio en el timepo
  private void EstimateModulePositions(SwerveModuleState[] states, double deltaTime) {
    modulesPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              modulesPositions[0].distanceMeters + (states[0].speedMetersPerSecond * deltaTime),
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[0].angle.getDegrees())),
          new SwerveModulePosition(
              modulesPositions[1].distanceMeters + (states[1].speedMetersPerSecond * deltaTime),
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[1].angle.getDegrees())),
          new SwerveModulePosition(
              modulesPositions[2].distanceMeters + (states[2].speedMetersPerSecond * deltaTime),
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[2].angle.getDegrees())),
          new SwerveModulePosition(
              modulesPositions[3].distanceMeters + (states[3].speedMetersPerSecond * deltaTime),
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[3].angle.getDegrees())),
        };
  }

  // Funcion para calcular los module positions (desde nuestros estados)
  private void EstimateModulePositions(SwerveModuleState[] states) {
    modulesPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              states[0].distanceMeters,
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[0].angle.getDegrees())),
          new SwerveModulePosition(
              states[1].distanceMeters,
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[1].angle.getDegrees())),
          new SwerveModulePosition(
              states[2].distanceMeters,
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[2].angle.getDegrees())),
          new SwerveModulePosition(
              states[3].distanceMeters,
              edu.wpi.first.math.geometry.Rotation2d.fromDegrees(states[3].angle.getDegrees())),
        };
  }

  // Funcion para agregar una medicion de la vision a la odometria
  public void addVisionMeasurement(double time, Pose2d pose) {
    poseEstimator.addVisionMeasurement(
        new edu.wpi.first.math.geometry.Pose2d(
            pose.getTranslation().x(),
            pose.getTranslation().y(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(pose.getRotation().getDegrees())),
        time);
  }

  // Funcion para actualizar la odometria en base a lso encoders y el tiempo
  public void updateWithTime(
      double time,
      double deltaTime,
      double robotAngle,
      SwerveModuleState[] states,
      ChassisSpeeds speeds) {
    m_velocity = speeds;
    EstimateModulePositions(states, deltaTime);
    poseEstimator.updateWithTime(
        time, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(robotAngle), modulesPositions);
  }

  // Funcion para leer las velocidades del chassis
  public ChassisSpeeds getVelocity() {
    return m_velocity;
  }

  // Funcion para leer la posicion de la odometria
  public Pose2d getEstimatedPosition() {
    var measure = poseEstimator.getEstimatedPosition();
    return new Pose2d(
        measure.getTranslation().getX(),
        measure.getTranslation().getY(),
        Rotation2d.fromDegrees(measure.getRotation().getDegrees()));
  }

  // Funcion para restear la odometria
  public void resetPosition(Pose2d pose) {
    m_velocity = new ChassisSpeeds();
    modulesPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
          new SwerveModulePosition(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
          new SwerveModulePosition(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
          new SwerveModulePosition(0, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
        };
    poseEstimator.resetPosition(
        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(pose.getRotation().getDegrees()),
        modulesPositions,
        new edu.wpi.first.math.geometry.Pose2d(
            pose.getTranslation().x(),
            pose.getTranslation().y(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(pose.getRotation().getDegrees())));
  }

  @Override
  public String toString() {
    return getEstimatedPosition().toString() + " - " + m_velocity.toString();
  }

  @Override
  public String toCSV() {
    return getEstimatedPosition().toCSV() + "," + m_velocity.toCSV();
  }

  @Override
  public String toCSVHeader() {
    StringBuilder builder = new StringBuilder();
    builder.append(getEstimatedPosition().toCSVHeader());
    builder.append(",");
    builder.append(m_velocity.toCSVHeader());

    return builder.toString();
  }
}
