package com.team3478.lib.control;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para calcular errores entre la data real y la predicha.
//              Usada principalmente para el pathplanner.
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Rotation2d;
import com.team3478.lib.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public class ErrorTracker {
  List<Pose2d> tracking_error_over_time_;
  int max_num_samples_;

  public ErrorTracker(int max_num_samples) {
    max_num_samples_ = max_num_samples;
    tracking_error_over_time_ = new ArrayList<Pose2d>(max_num_samples);
  }

  public void addObservation(Pose2d error) {
    if (tracking_error_over_time_.size() > max_num_samples_) {
      tracking_error_over_time_.remove(0);
    }
    tracking_error_over_time_.add(error);
  }

  public void reset() {
    tracking_error_over_time_.clear();
  }

  public Translation2d getMaxTranslationError() {
    if (tracking_error_over_time_.isEmpty()) return Translation2d.identity();
    double max_norm = Double.NEGATIVE_INFINITY;
    Translation2d max = null;
    for (var error : tracking_error_over_time_) {
      double norm = error.getTranslation().norm2();
      if (norm > max_norm) {
        max_norm = norm;
        max = error.getTranslation();
      }
    }
    return max;
  }

  public Rotation2d getMaxRotationError() {
    if (tracking_error_over_time_.isEmpty()) return Rotation2d.identity();
    double max_norm = Double.NEGATIVE_INFINITY;
    Rotation2d max = null;
    for (var error : tracking_error_over_time_) {
      double norm = Math.abs(error.getRotation().getRadians());
      if (norm > max_norm) {
        max_norm = norm;
        max = error.getRotation();
      }
    }
    return max;
  }

  public double getTranslationRMSE() {
    double error_sum = 0.0;
    for (var error : tracking_error_over_time_) {
      error_sum += error.getTranslation().norm2();
    }
    error_sum /= (double) tracking_error_over_time_.size();
    return Math.sqrt(error_sum);
  }

  public double getRotationRMSE() {
    double error_sum = 0.0;
    for (var error : tracking_error_over_time_) {
      error_sum += error.getRotation().getRadians() * error.getRotation().getRadians();
    }
    error_sum /= (double) tracking_error_over_time_.size();
    return Math.sqrt(error_sum);
  }

  public void printSummary() {
    if (tracking_error_over_time_.isEmpty()) return;
    LoggerManager.log(LogLevel.INFO, "Error Summary---");
    LoggerManager.log(
        LogLevel.INFO,
        "Translation: RMSE " + getTranslationRMSE() + ", Max: " + getMaxTranslationError());
    LoggerManager.log(
        LogLevel.INFO, "Rotation: RMSE " + getRotationRMSE() + ", Max: " + getMaxRotationError());
  }
}
