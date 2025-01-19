package com.team3478.frc2025;

///////////////////////////////////////////////////////////////////////////////
// Description: Interface para asegurar los susbsistemas cuenten con estados.
// Authors: -
// Notes:
//  -
///////////////////////////////////////////////////////////////////////////////

public interface StateHandler<T extends Enum<T>> {
  void setState(T state);

  T getState();

  // Funcion para cambiar el estado con un int
  default void setState(int _state) {
    T[] enumConstants = getState().getDeclaringClass().getEnumConstants();
    if (_state >= 0 && _state < enumConstants.length) {
      setState(enumConstants[_state]);
    }
  }
}
