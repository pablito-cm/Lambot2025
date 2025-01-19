package com.team3478.frc2025.subsystems;

///////////////////////////////////////////////////////////////////////////////
// Description: Esta clase es la encargada de leer la informacion de las limelights desde las
// networktables.
// Authors: Paola, Pablo
// Notes:
//  -
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Rotation2d;
import com.team3478.lib.loops.ILooper;
import com.team3478.lib.loops.Loop;
import com.team3478.lib.util.LimelightHelpers;
import com.team3478.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import lambotlogs.IAutoLogger;
import lambotlogs.SBInfo;

public class Network extends Subsystem implements IAutoLogger {

  // Instancia única de la clase
  private static Network mInstance;

  // Esta funcion es para regresar la instancia unica del subsistema (singleton)
  // @return {Network} instancia unica de la clase
  public static synchronized Network getInstance() {
    if (mInstance == null) {
      mInstance = new Network();
    }
    return mInstance;
  }

  // Para almacenar los inputs/outputs del subsistema
  public PeriodicIO mPeriodicIO;
  // Bandera para desactivar los logs de la smartdashboard.
  private boolean logToDashboard = false;

  // Clase para declarar las variables con los inputs/outputs default del subsistema
  public class PeriodicIO {
    // INPUTS
    @SBInfo("IGNORE")
    public double timestamp = 0;

    @SBInfo("TESTING")
    public double deltaTime = 0;
  }

  // Constructor de la clase
  private Network() {
    mPeriodicIO = new PeriodicIO();
  }

  public void readPeriodicInputs() {
    // Calculate time change
    double tempTime = Timer.getFPGATimestamp();
    mPeriodicIO.deltaTime = tempTime - mPeriodicIO.timestamp;
    mPeriodicIO.timestamp = tempTime;
  }

  public void writePeriodicOutputs() {}

  public void registerEnabledLoops(ILooper in) {
    in.register(
        new Loop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Network.this) {
              stop();
            }
          }

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // Funcion para calcular la odometria en base a la camara
  // Notas:
  // - Para el angulos usamos siempre el navx ya que es mas preciso
  // - El robot consideramos x positiva adelante
  // - El robot consideramos y positiva izquierda
  public Pose2d GetRobotOdometry(double robotAngle, double angleRate) {
    Pose2d newPose = null;

    if (Util.isRedAllience()) {
      robotAngle = Util.Limit360Angle(robotAngle + 180);
    }

    // Update Imu values so the limelight can use that in the megatag2 estimation to solve flipping
    // tag problem
    LimelightHelpers.SetRobotOrientation("limelight-back", robotAngle, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-front", robotAngle, 0, 0, 0, 0, 0);

    boolean rejectUpdate = false;
    LimelightHelpers.PoseEstimate measure =
        LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-back");
    // If robot is rotating fast reject update
    if (Math.abs(angleRate) > 360) { // degrees per second
      rejectUpdate = true;
    }
    // Validate that number of tags used to compose the pose is at least 1
    if (measure.tagCount == 0) {
      rejectUpdate = true;
    }
    // Evitamos leer tags a mas de 6 metros
    if (!rejectUpdate) {
      double distance = 0;
      int count = 0;
      for (var tag : measure.rawFiducials) {
        distance += Math.abs(tag.distToRobot);
        count++;
      }
      if ((distance / count) >= 6) {
        rejectUpdate = true;
      }
    }
    if (!rejectUpdate) {
      double posX = measure.pose.getTranslation().getX();
      double posY = measure.pose.getTranslation().getY();
      double posAngle = Util.Limit360Angle(robotAngle - 180);
      if (Util.isBlueAllience()) {
        posX = 17.5482504 - measure.pose.getTranslation().getX();
        posY = measure.pose.getTranslation().getY() * -1;
        posAngle = robotAngle;
      }
      newPose = new Pose2d(posX, posY, Rotation2d.fromDegrees(posAngle));
      return newPose;
    }

    rejectUpdate = false;
    measure = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-front");
    // LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-front");
    if (Math.abs(angleRate) > 360) { // degrees per secong
      rejectUpdate = true;
    }
    // Validate that number of tags used to compose the pose is at least 1
    if (measure.tagCount == 0) {
      rejectUpdate = true;
    }
    // Evitamos leer tags a mas de 6 metros
    if (!rejectUpdate) {
      double distance = 0;
      int count = 0;
      for (var tag : measure.rawFiducials) {
        distance += Math.abs(tag.distToRobot); // eucladian distance
        count++;
      }
      if ((distance / count) >= 6) {
        rejectUpdate = true;
      }
    }
    if (!rejectUpdate) {
      double posX = measure.pose.getTranslation().getX();
      double posY = measure.pose.getTranslation().getY();
      double posAngle = Util.Limit360Angle(robotAngle - 180);
      if (Util.isBlueAllience()) {
        posX = 17.5482504 - measure.pose.getTranslation().getX();
        posY = measure.pose.getTranslation().getY() * -1;
        posAngle = robotAngle;
      }
      newPose = new Pose2d(posX, posY, Rotation2d.fromDegrees(posAngle));
      return newPose;
    }

    return newPose;
  }

  public void stop() {}

  public boolean checkSystem() {
    return true;
  }

  // Funcion para enviar la data que queremos metricas
  @Override
  public List<Object> dataMetrics() {
    return List.of(mPeriodicIO);
  }

  // Funcion para enviar la data que queremos en el smartdatshboar
  @Override
  public List<Object> dataSmartDashboard() {
    if (!logToDashboard) return null;
    return List.of(mPeriodicIO);
  }
}
