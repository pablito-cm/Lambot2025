package com.team3478.frc2025.subsystems;

///////////////////////////////////////////////////////////////////////////////
// Description: Subsistema para controlar informacion global.
// networktables.
// Authors: -
// Notes:
//  - Este es utilizado tambien para guardar las metricas de informacion global.
///////////////////////////////////////////////////////////////////////////////

import com.team3478.lib.loops.ILooper;
import com.team3478.lib.loops.Loop;
import com.team3478.lib.util.Util;
import java.util.List;
import lambotlogs.IAutoLogger;

public class StatesSystem extends Subsystem implements IAutoLogger {

  // Instancia única de la clase
  private static StatesSystem mInstance;

  // Esta funcion es para regresar la instancia unica del subsistema (singleton)
  // @return {StatesSystem} instancia unica de la clase
  public static synchronized StatesSystem getInstance() {
    if (mInstance == null) {
      mInstance = new StatesSystem();
    }
    return mInstance;
  }

  // Para almacenar los inputs/outputs del subsistema
  public PeriodicIO mPeriodicIO;
  private int robotState = 0;
  // Bandera para desactivar los logs de la smartdashboard.
  private boolean logToDashboard = true;

  // Clase para declarar las variables con los inputs/outputs default del subsistema
  public class PeriodicIO {
    public boolean is_blue_allience = false;
  }

  // Constructor de la clase
  private StatesSystem() {
    mPeriodicIO = new PeriodicIO();
  }

  public void readPeriodicInputs() {
    mPeriodicIO.is_blue_allience = Util.isBlueAllience();
  }

  public void writePeriodicOutputs() {}

  public void registerEnabledLoops(ILooper in) {
    in.register(
        new Loop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (StatesSystem.this) {
              stop();
            }
          }

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  public void UpdateRobotState(int state) {
    robotState = state;
  }

  public int GetRobotState() {
    return robotState;
  }

  public void stop() {}

  public boolean checkSystem() {
    return true;
  }

  // Funcion para enviar la data que queremos metricas
  @Override
  public List<Object> dataMetrics() {
    return List.of(mPeriodicIO);
  }

  // Funcion para enviar la data que queremos en el smartdatshboar
  @Override
  public List<Object> dataSmartDashboard() {
    if (!logToDashboard) return null;
    return List.of(mPeriodicIO);
  }
}
