package com.team3478.lib.motion;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  * A MotionSegment is a movement from a start MotionState to an end MotionState with a constant
// acceleration.
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team3478/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import static com.team3478.lib.motion.MotionUtil.kEpsilon;
import static com.team3478.lib.util.Util.epsilonEquals;

import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public class MotionSegment {
  protected MotionState mStart;
  protected MotionState mEnd;

  public MotionSegment(MotionState start, MotionState end) {
    mStart = start;
    mEnd = end;
  }

  /**
   * Verifies that:
   *
   * <p>1. All segments have a constant acceleration.
   *
   * <p>2. All segments have monotonic position (sign of velocity doesn't change).
   *
   * <p>3. The time, position, velocity, and acceleration of the profile are consistent.
   */
  public boolean isValid() {
    if (!epsilonEquals(start().acc(), end().acc(), kEpsilon)) {
      // Acceleration is not constant within the segment.
      LoggerManager.log(
          LogLevel.ERROR,
          "Segment acceleration not constant! Start acc: "
              + start().acc()
              + ", End acc: "
              + end().acc());
      return false;
    }
    if (Math.signum(start().vel()) * Math.signum(end().vel()) < 0.0
        && !epsilonEquals(start().vel(), 0.0, kEpsilon)
        && !epsilonEquals(end().vel(), 0.0, kEpsilon)) {
      // Velocity direction reverses within the segment.
      LoggerManager.log(
          LogLevel.ERROR,
          "Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
      return false;
    }
    if (!start().extrapolate(end().t()).equals(end())) {
      // A single segment is not consistent.
      if (start().t() == end().t() && Double.isInfinite(start().acc())) {
        // One allowed exception: If acc is infinite and dt is zero.
        return true;
      }
      LoggerManager.log(
          LogLevel.ERROR, "Segment not consistent! Start: " + start() + ", End: " + end());
      return false;
    }
    return true;
  }

  public boolean containsTime(double t) {
    return t >= start().t() && t <= end().t();
  }

  public boolean containsPos(double pos) {
    return pos >= start().pos() && pos <= end().pos() || pos <= start().pos() && pos >= end().pos();
  }

  public MotionState start() {
    return mStart;
  }

  public void setStart(MotionState start) {
    mStart = start;
  }

  public MotionState end() {
    return mEnd;
  }

  public void setEnd(MotionState end) {
    mEnd = end;
  }

  @Override
  public String toString() {
    return "Start: " + start() + ", End: " + end();
  }
}
