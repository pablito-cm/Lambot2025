package com.team3478.frc2025.field;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para obtener y administrar las posiciones de las piezas
// Notas:
//  - Todo lo calibramos en base a la alianza roja considerando el 0,0 en la esquina inferior
// derecha
//  - Los ejes de referencia son x positivo adelante, y positivo izquierda, z positivo arriba,
// rotaciones positovas ccw
//  - Del lado azul la logica considera el 0,0 en la esquina inferiror izquierda igual (enfrente del
// punto rojo)
// con el mismo eje (por lo que las y seran negativas)
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Rotation2d;
import com.team3478.lib.geometry.Translation2d;
import com.team3478.lib.util.Util;
import java.util.Arrays;
import java.util.List;

public class GamePieces {

  // Enum de las posiciones clave de la cancha
  public static enum Pieces {
    NoValid,
    GamePiece1,
    GamePiece2,
    GamePiece3,
  }

  // Puntos con respecto a la alianza roja
  // Posiciones de todos los tags en la cancha, usar los angulos en radianes!
  private static Pose2d[] piecesmatrix =
      new Pose2d[] {
        new Pose2d(new Translation2d(0, 0), Rotation2d.identity()), // NoValid
        new Pose2d(new Translation2d(1.22, 2.197), Rotation2d.identity()), // GamePiece1
        new Pose2d(new Translation2d(1.22, 4.026), Rotation2d.identity()), // GamePiece2
        new Pose2d(new Translation2d(1.22, 5.855), Rotation2d.identity()), // GamePiece3
      };

  // Funcion para leer el pose de un tag
  public static Pose2d getPose(Pieces id) {
    return handleAllianceFlip(piecesmatrix[id.ordinal()]);
  }

  // Funcion para obtener cual es la pieza mas cercana a una posicion
  public static Pieces getNearestPiece(Pose2d pose, Pieces... pieces) {
    double nearest = Double.MAX_VALUE;
    double compare = 0;
    Pose2d piecePose = null;
    Pieces nearestPiece = Pieces.NoValid;
    if (pieces.length == 0) {
      List<Pieces> excludedPieces = Arrays.asList(Pieces.NoValid);
      for (Pieces piece : Pieces.values()) {
        if (excludedPieces.contains(piece)) {
          continue;
        }
        piecePose = getPose(piece);
        if (piece == null) {
          continue;
        }
        compare = piecePose.getTranslation().minus(pose.getTranslation()).norm();
        if (compare < nearest) {
          nearestPiece = piece;
          nearest = compare;
        }
      }
    } else {
      for (Pieces piece : pieces) {
        piecePose = getPose(piece);
        if (piece == null) {
          continue;
        }
        compare = piecePose.getTranslation().minus(pose.getTranslation()).norm();
        if (compare < nearest) {
          nearestPiece = piece;
          nearest = compare;
        }
      }
    }
    return nearestPiece;
  }

  // Funcion para ajustar los tags en base a la alianza
  private static Pose2d handleAllianceFlip(Pose2d point) {
    if (point == null) {
      return null;
    }
    if (Util.isBlueAllience()) {
      return new Pose2d(
          new Translation2d(point.getTranslation().x(), -point.getTranslation().y()),
          Rotation2d.identity());
    }
    return point;
  }
}
