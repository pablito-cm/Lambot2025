package com.team3478.frc2025;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team3478.frc2025.subsystems.Subsystem;
import com.team3478.lib.loops.ILooper;
import com.team3478.lib.loops.Loop;
import com.team3478.lib.loops.Looper;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Used to reset, start, stop, and update all subsystems at once */
public class SubsystemManager implements ILooper {
  private static SubsystemManager mInstance = null; // Singleton object

  private List<Subsystem> mAllSubsystems;
  private List<Loop> mLoops = new ArrayList<>();
  private List<BaseStatusSignal> canSignals = new ArrayList<>();
  private List<BaseStatusSignal> canCarnivoreSignals = new ArrayList<>();

  // Constructor de la clase
  private SubsystemManager() {}

  // Funcion para leer el singleton object (instancia unica de la clase)
  public static SubsystemManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemManager();
    }
    return mInstance;
  }

  // Funcion para agregar los BaseStatusSignal para actualizar los estados
  public void addStatusSignal(BaseStatusSignal signal) {
    canSignals.add(signal);
  }

  // Funcion para agregar los BaseStatusSignal para actualizar los estados
  public void addCarnivoreStatusSignal(BaseStatusSignal signal) {
    canCarnivoreSignals.add(signal);
  }

  // Funcion para llamar la funcion de checkSystem de cada subsistema
  public boolean checkSubsystems() {
    boolean ret_val = true;
    for (Subsystem s : mAllSubsystems) {
      ret_val &= s.checkSystem();
    }
    return ret_val;
  }

  // Funcion para llamar la funcion de stop de cada subsistema
  public void stop() {
    mAllSubsystems.forEach(Subsystem::stop); // usa el operador :: para pasar el metodo
  }

  // Funcion para cambiar todos los subsistemas a un subsistema en comun
  public void setState(int state) {
    for (Subsystem system : mAllSubsystems) {
      // Verificamos el subsistema cuente con estados
      if (system instanceof StateHandler) {
        StateHandler<?> stateHandler = (StateHandler<?>) system;
        stateHandler.setState(state);
      }
    }
  }

  // Funcion para leer la lista de subsistemas
  public List<Subsystem> getSubsystems() {
    return mAllSubsystems;
  }

  // Funcion para agregar elementos a la lista de subsistemas
  public void setSubsystems(Subsystem... allSubsystems) {
    mAllSubsystems = Arrays.asList(allSubsystems);
  }

  // Clase para crear el loop de enabled
  private class EnabledLoop implements Loop {
    @Override
    public void onStart(double timestamp) {
      mLoops.forEach(l -> l.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
      mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
      if (canSignals.size() > 0) {
        BaseStatusSignal.refreshAll(
            canSignals.toArray(new BaseStatusSignal[0])); // Update sensor status
      }
      if (canCarnivoreSignals.size() > 0) {
        BaseStatusSignal.refreshAll(
            canCarnivoreSignals.toArray(new BaseStatusSignal[0])); // Update sensor status
      }
      mLoops.forEach(l -> l.onLoop(timestamp));
      mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
      Robot.logger.update();
    }

    @Override
    public void onStop(double timestamp) {
      mLoops.forEach(l -> l.onStop(timestamp));
    }
  }

  // Clase para crear el loop de disabled
  private class DisabledLoop implements Loop {
    @Override
    public void onStart(double timestamp) {}

    @Override
    public void onLoop(double timestamp) {
      mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
      if (canSignals.size() > 0) {
        BaseStatusSignal.refreshAll(
            canSignals.toArray(new BaseStatusSignal[0])); // Update sensor status
      }
      if (canCarnivoreSignals.size() > 0) {
        BaseStatusSignal.refreshAll(
            canCarnivoreSignals.toArray(new BaseStatusSignal[0])); // Update sensor status
      }
    }

    @Override
    public void onStop(double timestamp) {}
  }

  // Funcion para registrar los subsistemas en el looper con la clase EnabledLoop
  public void registerEnabledLoops(Looper enabledLooper) {
    mAllSubsystems.forEach(
        s ->
            s.registerEnabledLoops(this)); // agrega cada subsistema al mLoops del Subsystem manager
    enabledLooper.register(new EnabledLoop());
  }

  // Funcion para registrar los subsistemas en el looper con la clase DisabledLoop
  public void registerDisabledLoops(Looper disabledLooper) {
    disabledLooper.register(new DisabledLoop());
  }

  // Funcion para regsitrar los loops de los subsitemas
  @Override
  public void register(Loop loop) {
    mLoops.add(loop);
  }
}
