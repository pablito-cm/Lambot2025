package com.team3478.frc2025.auto.actions;

///////////////////////////////////////////////////////////////////////////////
// Description:
//  * Composite action, running all sub-actions at the same time All actions are started then
// updated until all actions
//  * report being done.
// Authors: -
// Notes:
//  - Obtenido de: https://github.com/Team254/FRC-2023-Public
///////////////////////////////////////////////////////////////////////////////

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction implements Action {
  private final ArrayList<Action> mActions;

  public ParallelAction(List<Action> actions) {
    mActions = new ArrayList<>(actions);
  }

  public ParallelAction(Action... actions) {
    this(Arrays.asList(actions));
  }

  @Override
  public void start() {
    mActions.forEach(Action::start);
  }

  @Override
  public void update() {
    mActions.forEach(Action::update);
  }

  @Override
  public boolean isFinished() {
    for (Action action : mActions) {
      if (!action.isFinished()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void done() {
    mActions.forEach(Action::done);
  }
}
