package com.team3478.lib.drivers;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonUtil {
  /**
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkError(StatusCode errorCode, String message) {
    if (errorCode != StatusCode.OK) {
      DriverStation.reportError(message + " " + errorCode, false);
    }
  }

  /**
   * checks the specified error code and throws an exception if there are any issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkErrorWithThrow(StatusCode errorCode, String message) {
    if (errorCode != StatusCode.OK) {
      throw new RuntimeException(message + " " + errorCode);
    }
  }
}
