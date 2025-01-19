package com.team3478.frc2025.subsystems;

///////////////////////////////////////////////////////////////////////////////
// Description: Esta clase es la encargada del Drive del Swerve.
// Authors: Paola, Pablo
// Notes:
//  - Este archivo esta creado para el swerve de SDS
///////////////////////////////////////////////////////////////////////////////

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.team3478.frc2025.Constants;
import com.team3478.frc2025.StateHandler;
import com.team3478.frc2025.SubsystemManager;
import com.team3478.lib.drivers.TalonUtil;
import com.team3478.lib.geometry.Pose2d;
import com.team3478.lib.geometry.Rotation2d;
import com.team3478.lib.geometry.Translation2d;
import com.team3478.lib.loops.ILooper;
import com.team3478.lib.loops.Loop;
import com.team3478.lib.swerve.ChassisSpeeds;
import com.team3478.lib.swerve.SwerveDriveKinematics;
import com.team3478.lib.swerve.SwerveModuleState;
import com.team3478.lib.swerve.SwervePoseEstimator;
import com.team3478.lib.util.MotorUtil;
import com.team3478.lib.util.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import lambotlogs.IAutoLogger;
import lambotlogs.SBInfo;

public class Drive extends Subsystem implements IAutoLogger, StateHandler<Drive.State> {

  // Instancia única de la clase
  private static Drive mInstance;

  // Esta funcion es para regresar la instancia unica del subsistema (singleton)
  // @return {Drive} instancia unica de la clase
  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  // Es el sensor navx
  private AHRS navx;
  // Aqui se declaran los encoders para las steering wheel.
  private CANcoder frontRightabsoluteEncoder;
  private CANcoder frontLeftabsoluteEncoder;
  private CANcoder backRightabsoluteEncoder;
  private CANcoder backLeftabsoluteEncoder;
  // Son BaseStatusSignal para leer el valor de los encoders
  private BaseStatusSignal frontRightsignalEncoder;
  private BaseStatusSignal frontLeftsignalEncoder;
  private BaseStatusSignal backRightsignalEncoder;
  private BaseStatusSignal backLeftsignalEncoder;
  // Aqui se declaran los motores de speed.
  private TalonFX mFrontRightSpeedMotor,
      mFrontLeftSpeedMotor,
      mBackRightSpeedMotor,
      mBackLeftSpeedMotor;
  // Son BaseStatusSignal para leer la velocidad del motor
  private BaseStatusSignal mFrontRightSpeedMotorVelocitySignal;
  private BaseStatusSignal mFrontLeftSpeedMotorVelocitySignal;
  private BaseStatusSignal mBackRightSpeedMotorVelocitySignal;
  private BaseStatusSignal mBackLeftSpeedMotorVelocitySignal;
  // Aqui se declaran los motores de steering.
  private TalonFX mFrontRightSteeringMotor,
      mFrontLeftSteeringMotor,
      mBackRightSteeringMotor,
      mBackLeftSteeringMotor;

  // Estados internos del subsistema
  public static enum State {
    IDLE,
    Test,
    Driving
  }

  // Esta clase se usa para hacer calculos cinematicos con las velocidades de los motores.
  private MotorUtil mMotorUtilSpeed;
  // Objetos de controladores PID para alinear la rotacion del robot
  private PIDController robotYawPID, robotYawPIDdinamic;
  // Esta es la clase para la cinematica del Swerve
  private SwerveDriveKinematics mSwerveDriveKinematics;
  // Esta clase transforma las velocidades del robot (ene l eje de refrencia del robot)
  private ChassisSpeeds speeds;
  // Clase para guardar los estados de los modulos del swerve
  private SwerveModuleState[] moduleStates;
  // Clase para guardar la posicion del robot
  private SwervePoseEstimator odometry;
  // Variables para guardar los estados del robot
  private State mCurrentState = State.IDLE;
  private State mWantedState = State.IDLE;
  // Variables para guardar las posiciones de los modulos
  private Translation2d[] moduleLocations;
  // Variable para guardar el ultimo error de las steering wheels
  private double[] lastError = {0, 0, 0, 0};
  // Variables para almacenar los valores del control
  private double rawX, rawY, turn = 0;
  // para almacenar los inputs/outputs del subsistema
  public PeriodicIO mPeriodicIO;
  // Variables para el pathplanner
  // Bandera para desactivar los logs de la smartdashboard.
  private boolean logToDashboard = true;

  // Clase para declarar las variables con los inputs/outputs default del subsistema
  public static class PeriodicIO {
    // INPUTS
    @SBInfo("IGNORE")
    public double timestamp = 0;

    @SBInfo("TESTING")
    public double deltaTime = 0;

    @SBInfo("TESTING")
    public double yawAngle;

    @SBInfo("TESTING")
    public double yawAngleRate;

    @SBInfo("TESTING")
    public double yawAngleSetPoint = 0;

    @SBInfo("TESTING")
    public double quickStopAccumulator = 0;

    ////// variables para guardar los angulos de steering
    @SBInfo("TESTING")
    public double absFrontRightPosition;

    @SBInfo("TESTING")
    public double absFrontLeftPosition;

    @SBInfo("TESTING")
    public double absBackRightPosition;

    @SBInfo("TESTING")
    public double absBackLeftPosition;

    ///// variables para guardar las velocidades de los motores
    @SBInfo("TESTING")
    public double leftBackSpeed;

    @SBInfo("TESTING")
    public double leftFrontSpeed;

    @SBInfo("TESTING")
    public double rightBackSpeed;

    @SBInfo("TESTING")
    public double rightFrontSpeed;

    // Variable para saber el estado de los motores
    @SBInfo("IGNORE")
    public boolean breakModeOn = true;

    // OUTPUTS
    @SBInfo("TESTING")
    public double final_front_left_steering_motor;

    @SBInfo("TESTING")
    public double final_front_left_speed_motor;

    @SBInfo("TESTING")
    public double final_front_right_steering_motor;

    @SBInfo("TESTING")
    public double final_front_right_speed_motor;

    @SBInfo("TESTING")
    public double final_back_left_steering_motor;

    @SBInfo("TESTING")
    public double final_back_left_speed_motor;

    @SBInfo("TESTING")
    public double final_back_right_steering_motor;

    @SBInfo("TESTING")
    public double final_back_right_speed_motor;
  }

  // Constructor del subsystema
  private Drive() {
    mPeriodicIO = new PeriodicIO();
    mMotorUtilSpeed = new MotorUtil(Constants.Drive.kGearSpeedReduction);

    // Init Modules Positions
    moduleLocations = new Translation2d[Constants.Drive.kNumberOfModules];
    moduleLocations[0] =
        new Translation2d(Constants.Drive.kXFrontLeftLocation, Constants.Drive.kYFrontLeftLocation);
    moduleLocations[1] =
        new Translation2d(
            Constants.Drive.kXFrontRightLocation, Constants.Drive.kYFrontRightLocation);
    moduleLocations[2] =
        new Translation2d(Constants.Drive.kXBackRightLocation, Constants.Drive.kYBackRightLocation);
    moduleLocations[3] =
        new Translation2d(Constants.Drive.kXBackLeftLocation, Constants.Drive.kYBackLeftLocation);

    // Init Modules States
    moduleStates = new SwerveModuleState[Constants.Drive.kNumberOfModules];
    for (int i = 0; i < Constants.Drive.kNumberOfModules; i++) {
      moduleStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    // Init chassis speeds
    speeds = new ChassisSpeeds(0, 0, 0);

    // Init Swerve Kinematics object
    mSwerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);

    // Init odometry object
    ////// Set initial pose in 0,0 (el autonomo sobrescribe la posicion inicial
    ////// dependiendo el path que se ejecuta)
    odometry = new SwervePoseEstimator(0, 0, 0, moduleStates);

    // Init NavX
    try {
      navx =
          new AHRS(
              NavXComType.kMXP_SPI,
              50); // new AHRS(SPI.Port.kMXP, (byte) 50); // cambiando a 100 hertz el update time
      Timer.delay(0.5);
      navx.reset();
      Timer.delay(0.5);
    } catch (Exception e) {
      this.logError("navx not working");
    }

    // Init CAN Coders (para usar el carnivore)
    frontRightabsoluteEncoder = new CANcoder(Constants.Drive.kfrontRightCANCoderID, "canivore");
    frontLeftabsoluteEncoder = new CANcoder(Constants.Drive.kfrontLeftCANCoderID, "canivore");
    backRightabsoluteEncoder = new CANcoder(Constants.Drive.kbackRightCANCoderID, "canivore");
    backLeftabsoluteEncoder = new CANcoder(Constants.Drive.kbackLeftCANCoderID, "canivore");

    // Create cancoder configuration
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    configs.MagnetSensor.MagnetOffset = 0;
    configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // Apply cancoder configuration
    frontRightabsoluteEncoder.getConfigurator().apply(configs);
    frontLeftabsoluteEncoder.getConfigurator().apply(configs);
    backRightabsoluteEncoder.getConfigurator().apply(configs);
    backLeftabsoluteEncoder.getConfigurator().apply(configs);
    // Get cancoder signals and store in updater list
    frontRightsignalEncoder = frontRightabsoluteEncoder.getAbsolutePosition();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(frontRightsignalEncoder);
    frontLeftsignalEncoder = frontLeftabsoluteEncoder.getAbsolutePosition();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(frontLeftsignalEncoder);
    backRightsignalEncoder = backRightabsoluteEncoder.getAbsolutePosition();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(backRightsignalEncoder);
    backLeftsignalEncoder = backLeftabsoluteEncoder.getAbsolutePosition();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(backLeftsignalEncoder);

    // Inicializamos el pid para mantener el robot en el angulo deseado(yaw)
    robotYawPID =
        new PIDController(
            Constants.Drive.kPTurnValueSmallError, 0, Constants.Drive.kDTurnValueSmallError);
    robotYawPIDdinamic =
        new PIDController(
            Constants.Drive.kPTurnValueBigError, 0, Constants.Drive.kDTurnValueBigError);

    // Init Speed Motors
    mFrontRightSpeedMotor = new TalonFX(Constants.Drive.kFrontRightSpeedMotorID, "canivore");
    mFrontLeftSpeedMotor = new TalonFX(Constants.Drive.kFrontLeftSpeedMotorID, "canivore");
    mBackRightSpeedMotor = new TalonFX(Constants.Drive.kBackRightSpeedMotorID, "canivore");
    mBackLeftSpeedMotor = new TalonFX(Constants.Drive.kBackLeftSpeedMotorID, "canivore");

    // Init speed motors velocity sensor signal
    mFrontRightSpeedMotorVelocitySignal = mFrontRightSpeedMotor.getVelocity();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(mFrontRightSpeedMotorVelocitySignal);
    mFrontLeftSpeedMotorVelocitySignal = mFrontLeftSpeedMotor.getVelocity();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(mFrontLeftSpeedMotorVelocitySignal);
    mBackRightSpeedMotorVelocitySignal = mBackRightSpeedMotor.getVelocity();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(mBackRightSpeedMotorVelocitySignal);
    mBackLeftSpeedMotorVelocitySignal = mBackLeftSpeedMotor.getVelocity();
    SubsystemManager.getInstance().addCarnivoreStatusSignal(mBackLeftSpeedMotorVelocitySignal);

    // Init Steering Motors
    mFrontRightSteeringMotor = new TalonFX(Constants.Drive.kFrontRightSteeringMotorID, "canivore");
    mFrontLeftSteeringMotor = new TalonFX(Constants.Drive.kFrontLeftSteeringMotorID, "canivore");
    mBackRightSteeringMotor = new TalonFX(Constants.Drive.kBackRightSteeringMotorID, "canivore");
    mBackLeftSteeringMotor = new TalonFX(Constants.Drive.kBackLeftSteeringMotorID, "canivore");

    // Creamos las configuraciones para los falcon(talonfx) de speed
    TalonFXConfiguration talonSpeedMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs talonSpeedMotorConfigCurrent = new CurrentLimitsConfigs();
    talonSpeedMotorConfigCurrent.SupplyCurrentLimitEnable = true;
    talonSpeedMotorConfigCurrent.SupplyCurrentLimit = Constants.Drive.kSpeedMotorCurrentLimit;
    talonSpeedMotorConfigCurrent.SupplyCurrentLowerLimit = 0.2;
    talonSpeedMotorConfigCurrent.SupplyCurrentLowerLimit = 0;
    talonSpeedMotorConfig.CurrentLimits = talonSpeedMotorConfigCurrent;
    talonSpeedMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    TalonUtil.checkError(
        mFrontLeftSpeedMotor.getConfigurator().apply(talonSpeedMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mFrontLeftSpeedMotor");
    TalonUtil.checkError(
        mFrontRightSpeedMotor.getConfigurator().apply(talonSpeedMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mFrontRightSpeedMotor");
    TalonUtil.checkError(
        mBackLeftSpeedMotor.getConfigurator().apply(talonSpeedMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mBackLeftSpeedMotor");
    TalonUtil.checkError(
        mBackRightSpeedMotor.getConfigurator().apply(talonSpeedMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mBackRightSpeedMotor");

    // Creamos las configuraciones para los falcon(talonfx) de steering
    TalonFXConfiguration talonSteeringMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs talonSteeringMotorConfigCurrent = new CurrentLimitsConfigs();
    talonSteeringMotorConfigCurrent.SupplyCurrentLimitEnable = true;
    talonSteeringMotorConfigCurrent.SupplyCurrentLimit = Constants.Drive.kSteeringMotorCurrentLimit;
    talonSteeringMotorConfigCurrent.SupplyCurrentLowerLimit = 0.2;
    talonSteeringMotorConfigCurrent.SupplyCurrentLowerLimit = 0;
    talonSteeringMotorConfig.CurrentLimits = talonSteeringMotorConfigCurrent;
    TalonUtil.checkError(
        mFrontLeftSteeringMotor.getConfigurator().apply(talonSteeringMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mFrontLeftSteeringMotor");
    TalonUtil.checkError(
        mFrontRightSteeringMotor.getConfigurator().apply(talonSteeringMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mFrontRightSteeringMotor");
    TalonUtil.checkError(
        mBackLeftSteeringMotor.getConfigurator().apply(talonSteeringMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mBackLeftSteeringMotor");
    TalonUtil.checkError(
        mBackRightSteeringMotor.getConfigurator().apply(talonSteeringMotorConfig),
        "Drive-Drive-Error:Could not set configuration for mBackRightSteeringMotor");

    // Aseguramos el estado cambie driving al iniciar.
    setState(State.Driving);
  }

  // Metodo para leer los inputs (sensores, encoders..)
  public void readPeriodicInputs() {
    // Calculate time change
    double tempTime = Timer.getFPGATimestamp();
    mPeriodicIO.deltaTime = tempTime - mPeriodicIO.timestamp;
    mPeriodicIO.timestamp = tempTime;

    // Get navx angles (este es acumulativo)
    mPeriodicIO.yawAngle = navx.getAngle() * -1;
    mPeriodicIO.yawAngleRate = navx.getRate();

    // Se leen los angulos de steering de las ruedas convirtiendo de 0 a 360 grados y aplicando el
    // offset inicial de las ruedas.
    mPeriodicIO.absFrontRightPosition =
        (frontRightsignalEncoder.getValueAsDouble() * 360
            + Constants.Drive.kFrontRightEncoderInitPos);
    mPeriodicIO.absFrontLeftPosition =
        (frontLeftsignalEncoder.getValueAsDouble() * 360
            + Constants.Drive.kFrontLeftEncoderInitPos);
    mPeriodicIO.absBackRightPosition =
        (backRightsignalEncoder.getValueAsDouble() * 360
            + Constants.Drive.kBackRightEncoderInitPos);
    mPeriodicIO.absBackLeftPosition =
        (backLeftsignalEncoder.getValueAsDouble() * 360 + Constants.Drive.kBackLeftEncoderInitPos);
    // Se limitan los angulos de 0 a 360
    mPeriodicIO.absFrontRightPosition = Util.Limit360Angle(mPeriodicIO.absFrontRightPosition);
    mPeriodicIO.absFrontLeftPosition = Util.Limit360Angle(mPeriodicIO.absFrontLeftPosition);
    mPeriodicIO.absBackRightPosition = Util.Limit360Angle(mPeriodicIO.absBackRightPosition);
    mPeriodicIO.absBackLeftPosition = Util.Limit360Angle(mPeriodicIO.absBackLeftPosition);

    // Se calcula la velocidad lineal (m/s) de las ruedas de velocidad
    mPeriodicIO.rightBackSpeed =
        mBackRightSpeedMotorVelocitySignal.getValueAsDouble()
            * Math.PI
            * Constants.Drive.wheelRadius
            * 2
            * Constants.Drive.kGearSpeedReduction;
    mPeriodicIO.rightFrontSpeed =
        mFrontRightSpeedMotorVelocitySignal.getValueAsDouble()
            * Math.PI
            * Constants.Drive.wheelRadius
            * 2
            * Constants.Drive.kGearSpeedReduction;
    mPeriodicIO.leftBackSpeed =
        mBackLeftSpeedMotorVelocitySignal.getValueAsDouble()
            * Math.PI
            * Constants.Drive.wheelRadius
            * 2
            * Constants.Drive.kGearSpeedReduction;
    mPeriodicIO.leftFrontSpeed =
        mFrontLeftSpeedMotorVelocitySignal.getValueAsDouble()
            * Math.PI
            * Constants.Drive.wheelRadius
            * 2
            * Constants.Drive.kGearSpeedReduction;

    // Update odometry(usando los estados reales de las llantas)
    SwerveModuleState[] tempStates = new SwerveModuleState[Constants.Drive.kNumberOfModules];
    tempStates[0] =
        new SwerveModuleState(
            mPeriodicIO.leftFrontSpeed, Rotation2d.fromDegrees(mPeriodicIO.absFrontLeftPosition));
    tempStates[1] =
        new SwerveModuleState(
            mPeriodicIO.rightFrontSpeed, Rotation2d.fromDegrees(mPeriodicIO.absFrontRightPosition));
    tempStates[2] =
        new SwerveModuleState(
            mPeriodicIO.rightBackSpeed, Rotation2d.fromDegrees(mPeriodicIO.absBackRightPosition));
    tempStates[3] =
        new SwerveModuleState(
            mPeriodicIO.leftBackSpeed, Rotation2d.fromDegrees(mPeriodicIO.absBackLeftPosition));
    // Actualizamos la odometría en base a la cámara (si hay informacion)
    Pose2d cameraPose =
        Network.getInstance()
            .GetRobotOdometry(Util.Limit360Angle(mPeriodicIO.yawAngle), mPeriodicIO.yawAngleRate);
    if (cameraPose != null) {
      odometry.addVisionMeasurement(mPeriodicIO.timestamp, cameraPose);
    }
    // Actualizamos la odometria en base a los encoders
    odometry.updateWithTime(
        mPeriodicIO.timestamp,
        mPeriodicIO.deltaTime,
        Util.Limit360Angle(mPeriodicIO.yawAngle),
        tempStates,
        mSwerveDriveKinematics.toChassisSpeeds(tempStates));
  }

  // Metodo para setear los outputs de los motores
  public void writePeriodicOutputs() {
    mFrontRightSpeedMotor.set(mPeriodicIO.final_front_right_speed_motor);
    mFrontRightSteeringMotor.set(mPeriodicIO.final_front_right_steering_motor);

    mFrontLeftSpeedMotor.set(mPeriodicIO.final_front_left_speed_motor);
    mFrontLeftSteeringMotor.set(mPeriodicIO.final_front_left_steering_motor);

    mBackRightSpeedMotor.set(mPeriodicIO.final_back_right_speed_motor);
    mBackRightSteeringMotor.set(mPeriodicIO.final_back_right_steering_motor);

    mBackLeftSpeedMotor.set(mPeriodicIO.final_back_left_speed_motor);
    mBackLeftSteeringMotor.set(mPeriodicIO.final_back_left_steering_motor);
  }

  // Metodo para registrar los loops del subsistema
  public void registerEnabledLoops(ILooper in) {
    in.register(
        new Loop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Drive.this) {
              SetBrakeMode(true);
              stop();
            }
          }

          @Override
          public void onLoop(double timestamp) {
            LoopRobotStates();
            swerveMainDrive();
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // Funcion para actualizar los estados de los sticks en el drive
  // @param {double} _rawX: variable para actualizar el eje X del stick
  // @param {double} _rawY: variable para actualizar el eje Y del stick
  // @param {double} _turn: variable para la entrada del giro del robot
  // **DRIVER**
  public void swerveMainDrive(double _rawX, double _rawY, double _turn) {
    // Guardamos los valores de los sticks
    rawX = Util.handleDeadband(_rawX, 0.1) * -1;
    rawY = Util.handleDeadband(_rawY, 0.1);
    turn = Util.handleDeadband(_turn, 0.1) * -1;
  }

  // Funcion que corre la logica del drive (modo normal)
  public void swerveMainDrive() {
    // Seguridad para evitar que entre si no esta en el estado correcto
    if (mCurrentState != State.Driving) return;
    // Hacemos copia de los valores del stick para evitar que cambie
    // en medio de la ejecucion de la funcion (corren en loops separados)
    double _rawY = rawY;
    double _rawX = rawX;
    double _turn = turn;
    // Se calcula el error de la rotacion del robot
    double deltaAngle = mPeriodicIO.yawAngleSetPoint - mPeriodicIO.yawAngle;
    // Agregamos un acumulador de inercia para permitir el frenado rapido al girar el el propio eje
    // del robot
    boolean yesquickturn = Math.abs(_turn) > Constants.Drive.kDriveTurnDeadband;
    double acumulatorChange = Constants.Drive.kAcumulatorChange;
    double alpha = Constants.Drive.kAcumulatorAlpha;
    if (!yesquickturn) {
      _turn = _turn - mPeriodicIO.quickStopAccumulator;
      if (mPeriodicIO.quickStopAccumulator > acumulatorChange)
        mPeriodicIO.quickStopAccumulator -= acumulatorChange;
      else if (mPeriodicIO.quickStopAccumulator < -acumulatorChange)
        mPeriodicIO.quickStopAccumulator += acumulatorChange;
      else mPeriodicIO.quickStopAccumulator = 0.0;
    } else { // fast turns
      mPeriodicIO.quickStopAccumulator =
          (1 - alpha) * mPeriodicIO.quickStopAccumulator + alpha * _turn;
    }
    // Se calcula la salida del PID en base a la rotacion del robot y si el robot esta girando
    // actualiza el setpoint.
    if (Math.abs(_turn) < 1e-2) {
      if (Math.abs(deltaAngle) > 10) {
        _turn = robotYawPIDdinamic.calculate(mPeriodicIO.yawAngle, mPeriodicIO.yawAngleSetPoint);
      } else {
        _turn = robotYawPID.calculate(mPeriodicIO.yawAngle, mPeriodicIO.yawAngleSetPoint);
      }
    } else {
      mPeriodicIO.yawAngleSetPoint = mPeriodicIO.yawAngle;
    }
    // Se convierten los valores de los sticks a velocidad en m/s (van invertidos porque nuestro eje
    // de referencia apunta al frente del robot x positivo)
    double speedX = StickToVelocity(_rawY);
    double speedY = StickToVelocity(_rawX);
    // Convertir el valor de stick a velocidad angular
    double turnSpeed = StickToAngularVelocity(_turn);
    // Cambia los ejes de referencia de las velocidades entre la cancha al robot.
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speedX, speedY, turnSpeed, Rotation2d.fromDegrees((mPeriodicIO.yawAngle)));

    // Con Inverse Kinematics se convierten las velocidades a los estados de los modulos
    moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds);
    // Normalizar las velocidades para que no superen el máximo de las velocidades
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates,
        mMotorUtilSpeed.GetMaxVelocity(Constants.Drive.wheelRadius, Constants.Drive.kMotorRPM));
    // Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en
    // velocidad parar reducir el giro
    moduleStates[0] =
        SwerveModuleState.optimize(
            moduleStates[0], Rotation2d.fromDegrees(mPeriodicIO.absFrontLeftPosition));
    moduleStates[1] =
        SwerveModuleState.optimize(
            moduleStates[1], Rotation2d.fromDegrees(mPeriodicIO.absFrontRightPosition));
    moduleStates[2] =
        SwerveModuleState.optimize(
            moduleStates[2], Rotation2d.fromDegrees(mPeriodicIO.absBackRightPosition));
    moduleStates[3] =
        SwerveModuleState.optimize(
            moduleStates[3], Rotation2d.fromDegrees(mPeriodicIO.absBackLeftPosition));
    // Actualizar el estado de las salidas
    UpdateDriveSpeed();
  }

  // Funcion para leer el objeto de la odometria del drive
  public SwervePoseEstimator getOdometry() {
    return odometry;
  }

  // Funcion para actualizar los estados del subsistema
  private void LoopRobotStates() {
    if (mWantedState != mCurrentState) {
      switch (mCurrentState) {
        case IDLE:
          // nada
          break;
        case Driving:
          // nada
          break;
        case Test:
          // nada
          break;
      }
      mCurrentState = mWantedState;
      switch (mCurrentState) {
        case IDLE:
          // nada
          break;
        case Driving:
          // nada
          break;
        case Test:
          // nada
          break;
      }
    }
  }

  // Funcion para convertir el valor del stick a velocidad lineal
  // @param {double} stickInput: valor del stick a convertir
  private double StickToVelocity(double stickInput) {
    return stickInput
        * mMotorUtilSpeed.GetMaxVelocity(Constants.Drive.wheelRadius, Constants.Drive.kMotorRPM);
  }

  // Funcion para convertir el valor del stick a velocidad angular
  // @param {double} stickInput: valor del stick a convertir
  private double StickToAngularVelocity(double stickInput) {
    return stickInput
        * (mMotorUtilSpeed.GetMaxVelocity(Constants.Drive.wheelRadius, Constants.Drive.kMotorRPM)
            / (double) Constants.Drive.wheelTrack);
  }

  // Funcion para actualizar el estado de salidas
  private void UpdateDriveSpeed() {
    double deltadegrees = 0;
    // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
    if (Math.abs(moduleStates[0].speedMetersPerSecond) < 0.1) {
      mPeriodicIO.final_front_left_steering_motor = 0;
      mPeriodicIO.final_front_left_speed_motor = 0;
    } else {
      // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
      deltadegrees =
          Util.DeltaAngle(
              (double) Util.Limit360Angle(moduleStates[0].angle.getDegrees()),
              mPeriodicIO.absFrontLeftPosition);
      // Aplicamos el controlador PD para calcular la salida del motor steering
      mPeriodicIO.final_front_left_steering_motor =
          deltadegrees * Constants.Drive.kPSteeringValue
              + (deltadegrees - lastError[0])
                  / mPeriodicIO.deltaTime
                  * Constants.Drive.kDSteeringValue;
      // Actualizamos el ultimo error del motor de steering
      lastError[0] = deltadegrees;
      // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
      mPeriodicIO.final_front_left_speed_motor =
          mMotorUtilSpeed.GetPercentageFromVelocity(
              moduleStates[0].speedMetersPerSecond,
              Constants.Drive.wheelRadius,
              Constants.Drive.kMotorRPM);
    }
    // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
    if (Math.abs(moduleStates[1].speedMetersPerSecond) < 0.1) {
      mPeriodicIO.final_front_right_steering_motor = 0;
      mPeriodicIO.final_front_right_speed_motor = 0;
    } else {
      // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
      deltadegrees =
          Util.DeltaAngle(
              (double) Util.Limit360Angle(moduleStates[1].angle.getDegrees()),
              mPeriodicIO.absFrontRightPosition);
      // Aplicamos el controlador PD para calcular la salida del motor steering
      mPeriodicIO.final_front_right_steering_motor =
          (deltadegrees * Constants.Drive.kPSteeringValue)
              + (((deltadegrees - lastError[1]) / mPeriodicIO.deltaTime)
                  * Constants.Drive.kDSteeringValue);
      // Actualizamos el ultimo error del motor de steering
      lastError[1] = deltadegrees;
      // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
      mPeriodicIO.final_front_right_speed_motor =
          mMotorUtilSpeed.GetPercentageFromVelocity(
              moduleStates[1].speedMetersPerSecond,
              Constants.Drive.wheelRadius,
              Constants.Drive.kMotorRPM);
    }
    // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
    if (Math.abs(moduleStates[2].speedMetersPerSecond) < 0.1) {
      mPeriodicIO.final_back_right_steering_motor = 0;
      mPeriodicIO.final_back_right_speed_motor = 0;
    } else {
      // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
      deltadegrees =
          Util.DeltaAngle(
              (double) Util.Limit360Angle(moduleStates[2].angle.getDegrees()),
              mPeriodicIO.absBackRightPosition);
      // Aplicamos el controlador PD para calcular la salida del motor steering
      mPeriodicIO.final_back_right_steering_motor =
          deltadegrees * Constants.Drive.kPSteeringValue
              + (deltadegrees - lastError[2])
                  / mPeriodicIO.deltaTime
                  * Constants.Drive.kDSteeringValue;
      // Actualizamos el ultimo error del motor de steering
      lastError[2] = deltadegrees;
      // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
      mPeriodicIO.final_back_right_speed_motor =
          mMotorUtilSpeed.GetPercentageFromVelocity(
              moduleStates[2].speedMetersPerSecond,
              Constants.Drive.wheelRadius,
              Constants.Drive.kMotorRPM);
    }
    // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
    if (Math.abs(moduleStates[3].speedMetersPerSecond) < 0.1) {
      mPeriodicIO.final_back_left_steering_motor = 0;
      mPeriodicIO.final_back_left_speed_motor = 0;
    } else {
      // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
      deltadegrees =
          Util.DeltaAngle(
              (double) Util.Limit360Angle(moduleStates[3].angle.getDegrees()),
              mPeriodicIO.absBackLeftPosition);

      // Aplicamos el controlador PD para calcular la salida del motor steering
      mPeriodicIO.final_back_left_steering_motor =
          deltadegrees * Constants.Drive.kPSteeringValue
              + (deltadegrees - lastError[3])
                  / mPeriodicIO.deltaTime
                  * Constants.Drive.kDSteeringValue;
      // Actualizamos el ultimo error del motor de steering
      lastError[3] = deltadegrees;
      // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
      mPeriodicIO.final_back_left_speed_motor =
          mMotorUtilSpeed.GetPercentageFromVelocity(
              moduleStates[3].speedMetersPerSecond,
              Constants.Drive.wheelRadius,
              Constants.Drive.kMotorRPM);
    }
  }

  // Funcion para setear el estado del subsistema
  // @param {} _state: el estado del subsistema
  @Override
  public void setState(State _state) {
    if (mWantedState == _state) return;
    mWantedState = _state;
  }

  // Funcion para leer el estado del subsistema
  // @return {} mCurrentState: el estado del subsistema
  @Override
  public State getState() {
    return mCurrentState;
  }

  // Funcion para leer el estado del subsistema
  public State getWantedState() {
    return mWantedState;
  }

  // Funcion para cambiar el estado de los motores a brake.
  // @param {boolean} :Variable para habilitar o deshabilitar el estado de brake.
  public void SetBrakeMode(boolean enabled) {
    NeutralModeValue mode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    mFrontLeftSpeedMotor.setNeutralMode(mode);
    mFrontRightSpeedMotor.setNeutralMode(mode);
    mBackLeftSpeedMotor.setNeutralMode(mode);
    mBackRightSpeedMotor.setNeutralMode(mode);

    mFrontLeftSteeringMotor.setNeutralMode(mode);
    mFrontRightSteeringMotor.setNeutralMode(mode);
    mBackLeftSteeringMotor.setNeutralMode(mode);
    mBackRightSteeringMotor.setNeutralMode(mode);

    mPeriodicIO.breakModeOn = enabled;
  }

  // Funcion para detener todo el subsistema
  public void stop() {
    // Setea los estados de los sticks a 0
    rawX = 0;
    rawY = 0;
    turn = 0;
    // Setea la salida de los motores a 0
    mPeriodicIO.final_front_right_speed_motor = 0;
    mPeriodicIO.final_front_right_steering_motor = 0;
    mPeriodicIO.final_front_left_speed_motor = 0;
    mPeriodicIO.final_front_left_steering_motor = 0;
    mPeriodicIO.final_back_right_speed_motor = 0;
    mPeriodicIO.final_back_right_steering_motor = 0;
    mPeriodicIO.final_back_left_speed_motor = 0;
    mPeriodicIO.final_back_left_steering_motor = 0;
    // Aseguramos de mandar la salida po si se llama despues de matar el loop
    mFrontRightSpeedMotor.set(0);
    mFrontRightSteeringMotor.set(0);
    mFrontLeftSpeedMotor.set(0);
    mFrontLeftSteeringMotor.set(0);
    mBackRightSpeedMotor.set(0);
    mBackRightSteeringMotor.set(0);
    mBackLeftSpeedMotor.set(0);
    mBackLeftSteeringMotor.set(0);
  }

  // Funcion para restear el angulo de las ruedas de steering a 0
  // Necesita estar ligada a un boton the while held
  public void resetSteeringPositions() {
    if (mCurrentState != State.Test) {
      return;
    }
    moduleStates[0] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(0));
    moduleStates[1] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(0));
    moduleStates[2] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(0));
    moduleStates[3] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(0));
    // Actualizar el estado de las salidas
    UpdateDriveSpeed();
    // Aseguramos que no mande velocidad a las ruedas de speed
    mPeriodicIO.final_front_left_speed_motor = 0;
    mPeriodicIO.final_front_right_speed_motor = 0;
    mPeriodicIO.final_back_left_speed_motor = 0;
    mPeriodicIO.final_back_right_speed_motor = 0;
  }

  // Funcion para validar el subsistema
  public boolean checkSystem() {
    // TODO: implementar logica de testing
    return true;
  }

  // Funcion para enviar la data que queremos metricas
  @Override
  public List<Object> dataMetrics() {
    return List.of(
        mPeriodicIO,
        new Object() {
          @SuppressWarnings("unused")
          public String drive_state = mCurrentState.name();

          @SuppressWarnings("unused")
          public ChassisSpeeds chassis_speeds = speeds;

          @SuppressWarnings("unused")
          public SwerveModuleState swerve_module_state_FL = moduleStates[0];

          @SuppressWarnings("unused")
          public SwerveModuleState swerve_module_state_FR = moduleStates[1];

          @SuppressWarnings("unused")
          public SwerveModuleState swerve_module_state_BR = moduleStates[2];

          @SuppressWarnings("unused")
          public SwerveModuleState swerve_module_state_BL = moduleStates[3];

          @SuppressWarnings("unused")
          public SwervePoseEstimator swerve_odometry = odometry;

          @SuppressWarnings("unused")
          public Translation2d swerve_module_location_FL = moduleLocations[0];

          @SuppressWarnings("unused")
          public Translation2d swerve_module_location_FR = moduleLocations[1];

          @SuppressWarnings("unused")
          public Translation2d swerve_module_location_BR = moduleLocations[2];

          @SuppressWarnings("unused")
          public Translation2d sswerve_module_location_BL = moduleLocations[3];
        });
  }

  // Funcion para enviar la data que queremos en el smartdatshboar
  @Override
  public List<Object> dataSmartDashboard() {
    if (!logToDashboard) return null;
    return List.of(
        mPeriodicIO,
        new Object() {
          @SBInfo("TESTING")
          public String drive_state = mCurrentState.name();

          @SBInfo("TESTING")
          public ChassisSpeeds chassis_speeds = speeds;

          @SBInfo("TESTING,DRIVER")
          public SwervePoseEstimator swerve_odometry = odometry;
        });
  }
}
