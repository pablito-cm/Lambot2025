package com.team3478.lib.swerve;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  * Represents the speed of a robot chassis. Although this struct contains similar members
// compared
//  * to a Twist2d, they do NOT represent the same thing. Whereas a Twist2d represents a change in
// pose
//  * w.r.t to the robot frame of reference, this ChassisSpeeds struct represents a velocity w.r.t
// to
//  * the robot frame of reference.
//  *
//  * <p>A strictly non-holonomic drivetrain, such as a differential drive, should never have a dy
//  * component because it can never move sideways. Holonomic drivetrains such as swerve and mecanum
//  * will often have all three components.
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Rotation2d;
import com.team3478.lib.geometry.Twist2d;
import java.text.DecimalFormat;
import lambotlogs.csv.CSVWritable;

@SuppressWarnings("MemberName")
public class ChassisSpeeds implements CSVWritable {
  /** Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) */
  public double vxMetersPerSecond;

  /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
  public double vyMetersPerSecond;

  /** Represents the angular velocity of the robot frame. (CCW is +) */
  public double omegaRadiansPerSecond;

  /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
  public ChassisSpeeds() {}

  /**
   * Constructs a ChassisSpeeds object.
   *
   * @param vxMetersPerSecond Forward velocity.
   * @param vyMetersPerSecond Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   */
  public ChassisSpeeds(
      double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    this.vxMetersPerSecond = vxMetersPerSecond;
    this.vyMetersPerSecond = vyMetersPerSecond;
    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
   * object.
   *
   * @param vxMetersPerSecond The component of speed in the x direction relative to the field.
   *     Positive x is away from your alliance wall.
   * @param vyMetersPerSecond The component of speed in the y direction relative to the field.
   *     Positive y is to your left when standing behind the alliance wall.
   * @param omegaRadiansPerSecond The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      Rotation2d robotAngle) {
    return new ChassisSpeeds(
        vxMetersPerSecond * robotAngle.cos() + vyMetersPerSecond * robotAngle.sin(),
        -vxMetersPerSecond * robotAngle.sin() + vyMetersPerSecond * robotAngle.cos(),
        omegaRadiansPerSecond);
  }

  public static ChassisSpeeds fromRobotRelativeSpeeds(
      double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }

  public Twist2d toTwist2d() {
    return new Twist2d(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }

  @Override
  public String toString() {
    return String.format(
            "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond)
        .replace("-0.00", "0.00");
  }

  @Override
  public String toCSV() {
    final DecimalFormat format = new DecimalFormat("#0.000");
    return format.format(vxMetersPerSecond)
        + ","
        + format.format(vyMetersPerSecond)
        + ","
        + format.format(omegaRadiansPerSecond);
  }

  @Override
  public String toCSVHeader() {
    StringBuilder builder = new StringBuilder();
    builder.append("vxMetersPerSecond");
    builder.append(",");
    builder.append("vyMetersPerSecond");
    builder.append(",");
    builder.append("omegaRadiansPerSecond");

    return builder.toString();
  }
}
