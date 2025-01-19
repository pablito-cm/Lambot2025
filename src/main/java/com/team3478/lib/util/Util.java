package com.team3478.lib.util;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con funciones utiles generales.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.StringJoiner;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

/** Contains basic functions that are used often. */
public class Util {
  public static final double kEpsilon = 1e-12;

  /** Prevent this class from being instantiated. */
  private Util() {}

  /** Limits the given input to the given magnitude. */
  public static double limit(double v, double maxMagnitude) {
    return limit(v, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double v, double min, double max) {
    return Math.min(max, Math.max(min, v));
  }

  public static boolean inRange(double v, double maxMagnitude) {
    return inRange(v, -maxMagnitude, maxMagnitude);
  }

  /** Checks if the given input is within the range (min, max), both exclusive. */
  public static boolean inRange(double v, double min, double max) {
    return v > min && v < max;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  public static String joinStrings(final String delim, final List<?> strings) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < strings.size(); ++i) {
      sb.append(strings.get(i).toString());
      if (i < strings.size() - 1) {
        sb.append(delim);
      }
    }
    return sb.toString();
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
      result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
  }

  public static double handleDeadband(double value, double deadband) {
    deadband = Math.abs(deadband);
    if (deadband == 1) {
      return 0;
    }
    double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
    return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
  }

  public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public static String arrayToString(Object[] array, String delimeter) {
    StringJoiner joiner = new StringJoiner(delimeter);
    for (Object element : array) {
      joiner.add(element.toString());
    }
    return joiner.toString();
  }

  // Function to know if the alliecne color is blue
  public static boolean isBlueAllience() {
    if (!DriverStation.getAlliance().isPresent()) {
      LoggerManager.logfromlogger(LogLevel.ERROR, "Allience color not preset !!!");
      return false;
    }
    return DriverStation.getAlliance().get().toString() == DriverStation.Alliance.Blue.toString();
  }

  // Function to know if the alliecne color is red
  public static boolean isRedAllience() {
    if (!DriverStation.getAlliance().isPresent()) {
      LoggerManager.logfromlogger(LogLevel.ERROR, "Allience color not preset !!!");
      return false;
    }
    return DriverStation.getAlliance().get().toString() == DriverStation.Alliance.Red.toString();
  }

  // Funcion para genrar los filenames
  public static String generateFileName(String prefix, String extension) {
    LocalDateTime currentDateTime = LocalDateTime.now();
    // Define a custom date and time format
    DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
    // Format the current date and time using the formatter
    String formattedDateTime = currentDateTime.format(formatter);
    // Combine the formatted date and time with a file name or extension
    String fileName = prefix + "_" + formattedDateTime + "." + extension;
    return fileName;
  }

  // Funcion para obtener el delta mas corto entre dos angulos
  // Trabaja con angulos en el rango de 0-360
  // @param {double} double _target: angulo deseado
  // @param {double} double _actual: angulo actual
  public static double DeltaAngle(double _target, double _actual) {
    double deltadegrees = (_target - _actual);
    deltadegrees = ((deltadegrees + 180) - (Math.floor((deltadegrees + 180) / 360f) * 360)) - 180;
    return deltadegrees;
  }

  // Funcion que limita el angulo de 0 a 360
  // @param {double} double_angle: angulo que deseas limitar
  public static double Limit360Angle(double _angle) {
    _angle = _angle % 360;
    if (_angle < 0) {
      _angle += 360;
    }
    return _angle;
  }

  // Función para calcular el angulo entre 2 posiciones
  public static double calculateAngleBetweenPositions(Pose2d source, Pose2d target) {
    double angle =
        Math.atan2(
            target.getTranslation().y() - source.getTranslation().y(),
            target.getTranslation().x() - source.getTranslation().x());
    return angle;
  }

  // Function to dteermine if line intersect
  public static boolean lineIntersect(
      Translation2d origin, Translation2d newpoint, Translation2d P1, Translation2d P2) {
    double x1 = P1.x(), y1 = P1.y();
    double x2 = P2.x(), y2 = P2.y();

    double x3 = origin.x(), y3 = origin.y();
    double x4 = newpoint.x(), y4 = newpoint.y();

    // Calculate the denominator
    double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

    // Check if the lines are parallel
    if (denom == 0) return false;

    // Calculate the numerators
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    // Check if the intersection point is within the line segment and the vector's direction
    // Check if the intersection point is within both line segments
    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
      // // Calculate intersection point
      // double intersectionX = x1 + ua * (x2 - x1);
      // double intersectionY = y1 + ua * (y2 - y1);
      // System.out.println("Intersection point: (" + intersectionX + ", " + intersectionY + ")");
      return true;
    } else {
      return false;
    }
  }

  // Función para calcular el tiempo que le toma llegar la pieza al target considerando tiro
  // parabolico
  // Calcula el tiempo en x y el tiempo en y y los suma.
  public static double getTimeParabolicShoot(
      double x0, double y0, double x, double y, double vx, double vy) {
    return getTimeParabolicShootX(x0, x, vx) + getTimeParabolicShootY(y0, y, vy);
  }

  // Función para calcular el tiempo que le toma llegar la pieza al target considerando tiro
  // parabolico
  // Calcula el timepo en x.
  public static double getTimeParabolicShootX(double x0, double x, double vx) {
    // x=x0 + vx*t
    return (x - x0) / vx;
  }

  // Función para calcular el tiempo que le toma llegar la pieza al target considerando tiro
  // parabolico
  // Calcula el timepo en y.
  // Para resolver en y resuleve la ecuacion cuadrativa con la ecuacion -b+-root(b^2-4ac)/2a
  //  Pero solo usamos el termino de negativo ya que la pieza siempre entra de subida.
  //  (en tiro parabolico el +- es porque llega a la altura de subida y de bajada)
  public static double getTimeParabolicShootY(double y0, double y, double vy) {
    // y=y0+vy*t+1/2gt^2
    double a = -0.5 * 9.81;
    double b = vy;
    double c = y0 - y;
    double discriminant = ((b * b) - (4 * a * c));
    return (-b - discriminant) / (2 * a);
  }

  // Funcion para saber el angulo a ajustar en caso de querer disparar en movimeinto
  // Turret aim while moving
  // =======================

  // Law of sines:
  // sin A / a = sin B / b

  // Cross product of two vectors:
  // sin A = b x c / (|b| |c|)
  // sin B = a x c / (|a| |c|)
  // sin C = a x b / (|a| |b|)

  // Let g subscript be goal, b subscript be ball, t be time of flight of the ball, x
  // be position vector, v be velocity vector, and s be speed (norm of velocity
  // vector).

  // Instead of viewing it as drivetrain moving and target stationary, view it as
  // target moving and drivetrain stationary. Then the only velocities that matter
  // are ball and target, and the drivetrain position is just the starting point.
  // Therefore, v_g = -v_r.

  //        s_g t
  //       <----- x_g
  //      ^ A   B |
  //       \      |
  //        \     |
  //  s_b t  \    | d
  //          \   |
  //           \  |
  //            \C|
  //            x_b

  // Angle A is opposite of side a, angle B is opposite of side b, etc. x_b is
  // initial position vector of ball. We want to find angle C, which is the turret
  // heading adjustment from a moving target or drivetrain. "x" alone is cross
  // product.

  // Find equation for sin C.

  // sin B / s_b t = sin C / s_g t
  // sin B / s_b = sin C / s_g
  // sin C = sin B s_g / s_b

  // Find sin B.

  // sin B = a x c / (|a| |c|)
  // sin B = (x_g - x_b) x v_g / (s_g |x_g - x_b|)

  // Sub back in to equation for sin C.

  // sin C = (x_g - x_b) x v_g / (s_g |x_g - x_b|) * s_g / s_b
  // sin C = (x_g - x_b) x v_g / (|x_g - x_b| s_b)    ----> final equation

  // θ_t in global frame with no movement = atan2(x_g.y - x_r.y, x_g.x - x_r.x)
  // θ_t in drivetrain frame = θ_t in global - θ_r
  // θ_t adjustment = C
  // FunctionArguments: x_g: target position, x_b: shooter position, v_g: robot velocity, s_b: ball
  // velocity (magnitude)
  public static double getMovingRobotOffsetAngle(
      Translation2d x_g, Translation2d x_b, Translation2d v_r, double s_b) {
    // Calculate Target velocity
    Translation2d v_g = new Translation2d(v_r.x() * -1, v_r.y() * -1);
    // Calculate target velocity magnitude
    double s_g = Math.sqrt(v_g.x() * v_g.x() + v_g.y() * v_g.y());
    // Calculate sin B
    Translation2d a = new Translation2d(x_g.x() - x_b.x(), x_g.y() - x_b.y());
    double crossProduct = a.x() * v_g.y() - a.y() * v_g.x();
    double magnitude_a = Math.sqrt(a.x() * a.x() + a.y() * a.y());
    double magnitude_c = Math.sqrt(v_g.x() * v_g.x() + v_g.y() * v_g.y());
    double sinB = crossProduct / (magnitude_a * magnitude_c);
    // Calculate sin C
    double sinC = sinB * s_g / s_b;
    // Calculate angle C
    double angleC = Math.asin(sinC);
    // Convert angle from radians to degrees
    angleC = Math.toDegrees(angleC);
    return angleC;
  }

  public class LineIntersection {
    // Method to determine if two lines intersect
    public static boolean doLinesIntersect(
        int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
      // Calculate slopes
      double slope1 = getSlope(x1, y1, x2, y2);
      double slope2 = getSlope(x3, y3, x4, y4);
      // Check if lines are parallel (including the case of being coincident)
      if (slope1 == slope2) {
        // Check if they are coincident (overlapping)
        if (isCoincident(x1, y1, x2, y2, x3, y3, x4, y4)) {
          return true;
        }
        return false;
      }
      // Calculate intersection point
      double intersectX = getIntersectionX(x1, y1, x2, y2, x3, y3, x4, y4);
      double intersectY = getIntersectionY(x1, y1, x2, y2, x3, y3, x4, y4);
      // Check if intersection point is within the bounds of both segments
      boolean onLine1 = isPointOnLine(intersectX, intersectY, x1, y1, x2, y2);
      boolean onLine2 = isPointOnLine(intersectX, intersectY, x3, y3, x4, y4);
      if (onLine1 && onLine2) {
        return true;
      }
      return false;
    }

    // Helper method to calculate slope of a line
    private static double getSlope(int x1, int y1, int x2, int y2) {
      if (x1 == x2) {
        return Double.POSITIVE_INFINITY; // Vertical line
      }
      return ((double) (y2 - y1)) / (x2 - x1);
    }

    // Helper method to check if lines are coincident (overlapping)
    private static boolean isCoincident(
        int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
      return isPointOnLine(x1, y1, x3, y3, x4, y4)
          || isPointOnLine(x2, y2, x3, y3, x4, y4)
          || isPointOnLine(x3, y3, x1, y1, x2, y2)
          || isPointOnLine(x4, y4, x1, y1, x2, y2);
    }

    // Helper method to check if a point (px, py) lies on the line segment (x1, y1) to (x2, y2)
    private static boolean isPointOnLine(double px, double py, int x1, int y1, int x2, int y2) {
      if (px >= Math.min(x1, x2)
          && px <= Math.max(x1, x2)
          && py >= Math.min(y1, y2)
          && py <= Math.max(y1, y2)) {
        return true;
      }
      return false;
    }

    // Helper method to calculate x-coordinate of intersection point
    private static double getIntersectionX(
        int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
      double slope1 = getSlope(x1, y1, x2, y2);
      double slope2 = getSlope(x3, y3, x4, y4);
      double intercept1 = y1 - slope1 * x1;
      double intercept2 = y3 - slope2 * x3;
      return (intercept2 - intercept1) / (slope1 - slope2);
    }

    // Helper method to calculate y-coordinate of intersection point
    private static double getIntersectionY(
        int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
      double slope1 = getSlope(x1, y1, x2, y2);
      double intercept1 = y1 - slope1 * x1;
      return slope1 * getIntersectionX(x1, y1, x2, y2, x3, y3, x4, y4) + intercept1;
    }
  }

  // Funcion para aplicar rotaciones en z (con una matriz de rotacion)
  public static Pose3d applyYawRotation(Pose3d _target, double rotation) {
    double cosYaw = Math.cos(rotation);
    double sinYaw = Math.sin(rotation);
    double newX =
        _target.getTranslation().getX() * cosYaw - _target.getTranslation().getY() * sinYaw;
    double newY =
        _target.getTranslation().getX() * sinYaw + _target.getTranslation().getY() * cosYaw;
    return new Pose3d(
        new Translation3d(newX, newY, _target.getTranslation().getZ()), _target.getRotation());
  }
}
