package com.team3478.frc2025.field;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para obtener y administrar la posiciones de lso elementos de la cancha.
// Notas:
//  - Todo lo calibramos en base a la alianza roja considerando el 0,0 en la esquina inferior
// derecha
//  - Los ejes de referencia son x positivo adelante, y positivo izquierda, z positivo arriba,
// rotaciones positovas ccw
//  - La logica considera el 0,0 en la esquina inferiror izquierda igual (enfrente del punto rojo)
// con el mismo eje (por lo que las y seran negativas)
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.util.Util;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldElementPositions {

  // Enum de las posiciones clave de la cancha
  public static enum Element {
    FeederRight,
    FeederLeft,
    Reef_1,
    Reef_2,
    Reef_3,
    Reef_4,
    Reef_5,
    Reef_6
  }

  // Puntos con respecto a la alianza roja
  // Posiciones de todos los tags en la cancha, usar los angulos en radianes!
  private static Pose3d[] posematrix =
      new Pose3d[] {
        new Pose3d(
            new Translation3d(0.85, 7.4, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(-54))), // FeederRight
        new Pose3d(
            new Translation3d(0.85, 0.65, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(54))), // FeederLeft
        new Pose3d(
            new Translation3d(3.66, 4.03, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(0))), // Reef_1
        new Pose3d(
            new Translation3d(4.08, 3.32, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(-60))), // Reef_2
        new Pose3d(
            new Translation3d(4.91, 3.32, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(-120))), // Reef_3
        new Pose3d(
            new Translation3d(5.32, 4.03, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(180))), // Reef_4
        new Pose3d(
            new Translation3d(4.91, 4.75, 0), new Rotation3d(0, 0, Units.degreesToRadians(120))), // Reef_5
        new Pose3d(
            new Translation3d(4.08, 4.75, 0),
            new Rotation3d(0, 0, Units.degreesToRadians(60))), // Reef_6
      };

  // Funcion para leer el pose de la posicion deseada
  public static Pose3d getPose(Element _element) {
    return handleAllianceFlip(posematrix[_element.ordinal()]);
  }

  // Funcion para actualizar un pose de ser necesario
  public static void setPose(Element _element, Pose3d pose) {
    posematrix[_element.ordinal()] = pose;
  }

  private static Pose3d handleAllianceFlip(Pose3d point) {
    if (point == null) {
      return null;
    }
    if (Util.isBlueAllience()) {
      return new Pose3d(
          new Translation3d(
              point.getTranslation().getX(),
              -point.getTranslation().getY(),
              point.getTranslation().getZ()),
          new Rotation3d(
              point.getRotation().getX(),
              point.getRotation().getY(),
              Units.degreesToRadians(
                  Util.Limit360Angle(180 + Units.radiansToDegrees(point.getRotation().getZ())))));
    }
    return point;
  }
}
