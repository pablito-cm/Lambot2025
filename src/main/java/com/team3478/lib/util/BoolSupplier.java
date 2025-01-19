package com.team3478.lib.util;

import java.util.function.BooleanSupplier;

public class BoolSupplier implements BooleanSupplier {
  private boolean state = false;

  public BoolSupplier set(boolean _state) {
    state = _state;
    return this;
  }

  public boolean getAsBoolean() {
    return state;
  }
}
