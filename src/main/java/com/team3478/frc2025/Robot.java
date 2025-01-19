package com.team3478.frc2025;

///////////////////////////////////////////////////////////////////////////////
// Description: Archivo donde se inicializan los loops del robot y se crean los links a los
//              subsistemas.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.auto.AutoModeExecutor;
import com.team3478.frc2025.auto.AutoModeSelector;
import com.team3478.frc2025.auto.modes.AutoModeBase;
import com.team3478.frc2025.controlboard.ControlBoard;
import com.team3478.frc2025.subsystems.Drive;
import com.team3478.frc2025.subsystems.Rollers;
import com.team3478.frc2025.subsystems.Hanger;
import com.team3478.frc2025.subsystems.Network;
import com.team3478.frc2025.subsystems.StatesSystem;
import com.team3478.lib.loops.Looper;
import com.team3478.lib.util.Util;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import lambotlogs.Logger;
import lambotlogs.logger.LoggerManager;
import lambotlogs.logger.LoggerManager.LogLevel;

public class Robot extends TimedRobot {

  // Creamos el objeto para hacer logs,metrics y salidas al smartdashboard
  public static Logger logger = null;

  // Crea los objectos para los loops de enabled y disabled
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();

  // Crea objetos para seleccionar autónomos y ejecutarlos respectivamente
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  private AutoModeExecutor mAutoModeExecutor = null;

  // Declaramos los subsistemas del robot
  private final ControlBoard mControlBoard;
  private final Drive mDrive;
  private final Network mNetwork;
  private final Rollers mRollers;
  private final Hanger mHanger;
  private final StatesSystem mStateSystem;

  // Clase encargada de los subsistemas y su administracion
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  // Para guardar el timestamp del fpga (y contar el tiempo que lleva deshabilitado el robot)
  private double mDisabledStartTime = Double.NaN;

  // Constructor de la clase para inicializar las instancias unicas de cada subsistema
  Robot() {
    super();
    // Inicializamos los subsistemas y el controlboard
    mControlBoard = ControlBoard.getInstance();
    mDrive = Drive.getInstance();
    mNetwork = Network.getInstance();
    mRollers = Rollers.getInstance();
    mHanger = Hanger.getInstance();
    mStateSystem = StatesSystem.getInstance();
    // Inicializamos el logger/metrics saver object
    LoggerManager.setLevel(Constants.klogLevel);
    logger =
        new Logger(
            Constants.kmetricMode,
            Constants.ksmartdahboardMode,
            "/home/lvuser/" + Util.generateFileName("metrics", "csv"),
            "/home/lvuser/" + Util.generateFileName("logs", "txt"));
    LoggerManager.logfromlogger(LogLevel.INFO, "Robot Startup Correctly");
  }

  // Esta funcion coree una sola vez cuando el robot es prendido (o se le carga un codigo)
  @Override
  public void robotInit() {
    try {
      // TODO: Quitar el comentario a esta linea para activar la camara usb que usaremos para el que
      // driver ve las piezas que comera
      // CameraServer.startAutomaticCapture(); //se inicia la captura de la USB camera

      // Agrega la lista de subsystemas al subsystemManager
      mSubsystemManager.setSubsystems(mDrive, mStateSystem, mHanger, mRollers);

      // Agregamos los sources al logger de los cuales sacar informacion
      logger.addSources(mDrive, mRollers, mHanger, mStateSystem);

      // Registra los subsistemas a los loopers de enabled y disabled
      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      // Asegura que todos los subsistemas esten detenidos
      mSubsystemManager.stop();

      // Leemos el estado del auto selector para cargar el autonomo
      mAutoModeSelector.updateModeCreator();

    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  // Funcion que corre al iniciar el disable mode
  @Override
  public void disabledInit() {
    try {
      LoggerManager.logfromlogger(LogLevel.INFO, "Disable mode init");
      // Detenemos el loop de enable
      mEnabledLooper.stop();
      logger.stop();

      // Reset all auto mode state.
      if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }
      mAutoModeSelector.reset();
      mAutoModeSelector.updateModeCreator();
      mAutoModeExecutor = new AutoModeExecutor();

      // Inicializamos el loop de disabled
      mDisabledLooper.start();
      mDisabledStartTime = Timer.getFPGATimestamp();

    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  // Funcion que corre al iniciar el autonomos
  @Override
  public void autonomousInit() {
    try {
      LoggerManager.logfromlogger(LogLevel.INFO, "Auto mode init");
      mStateSystem.UpdateRobotState(1);
      // Detenemos el loop de disable
      mDisabledLooper.stop();
      // Inicializamos el loop de enable
      mEnabledLooper.start();
      logger.start();
      // Inicializamos el autonomo
      mAutoModeExecutor.start();

    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  // Funcion que se llama cuando inicia el teleoperado
  @Override
  public void teleopInit() {
    try {
      LoggerManager.logfromlogger(LogLevel.INFO, "Teleop mode init");
      mStateSystem.UpdateRobotState(2);
      // Detenemos el loop de disable
      mDisabledLooper.stop();
      // Aseguramos de parar el autonomo
      if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }
      // Asegura que todos los subsistemas esten detenidos
      mSubsystemManager.stop();
      // Inicializamos el loop de enable
      mEnabledLooper.start();
      logger.start();
      // Aseguramos el drive este en el estado para manejar
      mDrive.setState(Drive.State.Driving);
    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  // Funcion que se corre cuando inicia el ciclo de test
  @Override
  public void testInit() {
    try {
      LoggerManager.logfromlogger(LogLevel.INFO, "Test mode init");
      // Detenemos el loop de disable
      mDisabledLooper.stop();
      // Inicializamos el loop de enable
      mEnabledLooper.start();
      // En el modo test no activamos el logger
      logger.stop();
      // Cambiamos a todos los subsistemas al estado de test
      mSubsystemManager.setState(1);

    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
    try {
      // Send data to smarthdashboard
      logger.updateDashboard();
      mAutoModeSelector.outputToSmartDashboard();
    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  // Corre periodico mientras este deshabilitado
  @Override
  public void disabledPeriodic() {
    try {
      // Update auto modes AND LOAD TO EXECUTOR IF CHANGE DETECTED
      mAutoModeSelector.updateModeCreator();
      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
      if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
        LoggerManager.log(
            LogLevel.INFO, "Set auto mode to: " + autoMode.get().getClass().toString());
        mAutoModeExecutor.setAutoMode(autoMode.get());
      }

      // Una vez desabilitado cambias el chassis a coast
      if ((Timer.getFPGATimestamp() - mDisabledStartTime) > 5.0 && mDrive.mPeriodicIO.breakModeOn) {
        LoggerManager.log(LogLevel.INFO, "Setting drive to coast!");
        mDrive.SetBrakeMode(false);
      }
    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  // Corre periodico mientras este en autonomo
  @Override
  public void autonomousPeriodic() {}

  // Corre periodico mientras este habilitado
  @Override
  public void teleopPeriodic() {
    try {
      // All control board logic is here
      //// Driver
      mDrive.swerveMainDrive(
          mControlBoard.driveXVelocity(), // Stick left x
          mControlBoard.driveYVelocity(), // Stick left y
          mControlBoard.driveTurnVelocity() // Stick right x
          );
      mRollers.activar(mControlBoard.activate());
      mHanger.Subir(mControlBoard.up(), mControlBoard.down());
    } catch (Throwable t) {
      LoggerManager.logfromlogger(LogLevel.ERROR, t.getMessage());
      throw t;
    }
  }

  @Override
  public void testPeriodic() {
    // TODO: Hacer modo de testing en base a botones de los controles
  }
}
