package com.team3478.lib.loops;

///////////////////////////////////////////////////////////////////////////////
// Description: This code runs all of the robot's loops. Loop objects are stored in a List object.
// They are started when the robot powers up and stopped after the match.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.Constants;
import com.team3478.lib.util.CrashTrackingRunnable;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public class Looper implements ILooper {
  public final double kPeriod = Constants.kLooperDt; // dt del loop (cada cuanto corre)

  private boolean mRunning; // bandera para matener corriendo el sistema de loops

  private final Notifier mNotifier;
  // lista con los loops registrados (accedidos con un interface)
  private final List<Loop> mLoops;
  // creamos un objeto vacio para permitir que los syncronized blocks que tengan
  private final Object mTaskRunningLock = new Object();
  // este puedan correr en un tread diferente
  private double mTimestamp = 0;
  // variable para ir guardando el delta time de cada que corre todos los loops
  private double mDT = 0;

  // Creamos el objeto del runnable que va estar corriendo cada ciclo
  private final CrashTrackingRunnable runnable_ =
      new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
          synchronized (mTaskRunningLock) {
            if (mRunning) {
              double now = Timer.getFPGATimestamp();
              // Corre los loops registrados
              for (Loop loop : mLoops) {
                loop.onLoop(now);
              }
              // Guardamos cuanto tardo en correr los loops
              mDT = now - mTimestamp;
              mTimestamp = now;
            }
          }
        }
      };

  // Constructor de la clase
  public Looper() {
    mNotifier = new Notifier(runnable_); // notifier para invocar el roonable cada tiempo especifico
    mRunning = false;
    mLoops = new ArrayList<>();
  }

  // Funcion para agregar loops a la lista
  @Override
  public synchronized void register(Loop loop) {
    synchronized (mTaskRunningLock) {
      mLoops.add(loop);
    }
  }

  // Funcion para inicializar el ciclo de loops
  public synchronized void start() {
    if (!mRunning) {
      LoggerManager.log(LogLevel.INFO, "Starting loops");

      // Corre el onStart de los loops
      synchronized (mTaskRunningLock) {
        mTimestamp = Timer.getFPGATimestamp();
        for (Loop loop : mLoops) {
          loop.onStart(mTimestamp);
        }
        mRunning = true;
      }

      // Inicia el ciclo de correr los loops
      mNotifier.startPeriodic(kPeriod);
    }
  }

  // Funcion para detener los loops
  public synchronized void stop() {
    if (mRunning) {
      LoggerManager.log(LogLevel.INFO, "Stopping loops");
      mNotifier.stop(); // finaliza el notifier

      // Corre el onStop de los loops
      synchronized (mTaskRunningLock) {
        mRunning = false;
        mTimestamp = Timer.getFPGATimestamp();
        for (Loop loop : mLoops) {
          LoggerManager.log(LogLevel.INFO, "Stopping " + loop);
          loop.onStop(mTimestamp);
        }
      }
    }
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("looper_dt", mDT);
  }
}
