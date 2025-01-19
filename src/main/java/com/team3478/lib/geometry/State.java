package com.team3478.lib.geometry;

///////////////////////////////////////////////////////////////////////////////
// Description: -
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.util.Interpolable;
import lambotlogs.csv.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  S add(S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}
