package com.team3478.frc2025.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import lambotlogs.IAutoLogger;

public class Hanger extends Subsystem implements IAutoLogger {

    private static Hanger mInstance;

    // Esta funcion es para regresar la instancia unica del subsistema (singleton)
    // @return {Drive} instancia unica de la clase
    public static synchronized Hanger getInstance() {
      if (mInstance == null) {
        mInstance = new Hanger();
      }
      return mInstance;
    }

    static TalonFX MotorHanger;

    double speed;

    public Hanger(){
        MotorHanger = new TalonFX(9);
        speed = 0;
    }

    public void stop(){
        MotorHanger.set(0);
    }


    public void Subir(boolean getAButton, boolean getXButton){
        if(getAButton){
            speed = 0.5;
        }
        else if(getXButton){
            speed = -0.5;
        }
        else{
            speed = 0;
        }
        MotorHanger.set(speed);
    }

      // Funcion para validar el subsistema
  public boolean checkSystem() {
    return true;
  }
}
