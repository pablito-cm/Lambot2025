package com.team3478.lib.trajectory;

// ///////////////////////////////////////////////////////////////////////////////
// // Description:
// //  *
// // Authors: -
// // Notes:
// //  - Obtenido de: https://github.com/Team254/FRC-2023-Public
// ///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Twist2d;

public interface IPathFollower {
  Twist2d steer(Pose2d current_pose);

  boolean isDone();
}
