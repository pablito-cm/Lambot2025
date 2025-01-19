package com.team3478.frc2025.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team3478.frc2025.SubsystemManager;

public class Rollers {

    private static Rollers mInstance;

    // Esta funcion es para regresar la instancia unica del subsistema (singleton)
    // @return {Drive} instancia unica de la clase
    public static synchronized Rollers getInstance() {
      if (mInstance == null) {
        mInstance = new Rollers();
      }
      return mInstance;
    }

    static TalonSRX MotorRoller_1;
    static TalonSRX MotorRoller_2;

    double speed;

    public Rollers(){
    MotorRoller_1 = new TalonSRX(16);
    MotorRoller_2 = new TalonSRX(15);

    speed = 0;
    }

    public void stop(){
        MotorRoller_1.set(TalonSRXControlMode.PercentOutput,0);
        MotorRoller_2.set(TalonSRXControlMode.PercentOutput,0);
    }


    public void activar(boolean getBButton){
        if(getBButton){
            speed = .5;
        }
        else{
            speed = 0;
        }
        MotorRoller_1.set(TalonSRXControlMode.PercentOutput, speed);
        MotorRoller_2.set(TalonSRXControlMode.PercentOutput, -speed);
    }
}
