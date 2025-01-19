package com.team3478.lib.trajectory;

// ///////////////////////////////////////////////////////////////////////////////
// // Description:
// //  *
// // Authors: -
// // Notes:
// //  - Obtenido de: https://github.com/Team254/FRC-2023-Public
// ///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.geometry.State;

public interface TrajectoryView<S extends State<S>> {
  TrajectorySamplePoint<S> sample(final double interpolant);

  double first_interpolant();

  double last_interpolant();

  Trajectory<S> trajectory();
}
