package com.team3478.lib.loops;

///////////////////////////////////////////////////////////////////////////////
// Description: Interface for loops, which are routine that run periodically in the robot code (such
// as periodic gyroscope calibration, etc.)
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public interface Loop {
  void onStart(double timestamp);

  void onLoop(double timestamp);

  void onStop(double timestamp);
}
