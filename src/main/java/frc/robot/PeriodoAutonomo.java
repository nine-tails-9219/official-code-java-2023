package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.Timer;

public class PeriodoAutonomo {
    // Pigeon 2.0
    private final int ID_PIGEON = 9;
    private Pigeon2 pigeon2 = new Pigeon2(ID_PIGEON);
    private double angleRobot = 0;
    private double angleRobotRounded = 0;

    // Timer
    private final Timer esperaTimer = new Timer();

    // Interruptores usados no autônomo
    private boolean climb = false;
    private boolean stop = false;

    // ID Autonomo
    private int idAutonomo = 0;

    // Motores
    private MotoresTank motoresTank;

    public PeriodoAutonomo(MotoresTank motoresTank){
        this.motoresTank = motoresTank;
    }

    public void UpdatePigeon(){
        angleRobot = (pigeon2.getRoll() * -1) - 5;
        angleRobotRounded = (angleRobot * 100) / 100;
    }

    public void UpdateDashboardInfo(){
        // Variáveis usadas na ChargeStation() e ReverseAnd180()
        //SmartDashboard.putNumber("Roll", pigeon2.getRoll()); 
        //SmartDashboard.putNumber("Yaw (EixoX)", pigeon2.getYaw());
        //SmartDashboard.putNumber("Ângulo", angleRobotRounded);
        //SmartDashboard.putBoolean("Subiu", climb);
        //SmartDashboard.putBoolean("Parar", stop);
    }

    public void ResetPigeon(){
        pigeon2.setYaw(0);
        pigeon2.configFactoryDefault();
    }

    public void ResetTimer(){
        esperaTimer.restart();
    }

    public void ResetInterruptors(){
        climb = false;
        stop = false;
    }

    public void PeriodoAutonomo(){
        switch (idAutonomo) {
            case 1:
              //DeliverMiddleAndExitCommunity(); // Entrega no mid e sai da comunidade
              break;
      
            case 2: 
              CubeInLowAndChargeStation(); // Entrega o cubo no low e sobre na charge station
              break;
      
            case 3: 
              //DeliverMiddleAndReverseAnd180AndChargeStation();// Entrega no mid, chega para trás, vira 180° e sobe na charge station
              break;
          }
    }

    private void CubeInLowAndChargeStation(){

        while (esperaTimer.get() < 1.5) {
            motoresTank.SetTankDrive(0.3, 0.3);
        }

        while (esperaTimer.get() < 1.7) {
            motoresTank.SetTankDrive(-0.6, -0.6);
        }

        if (angleRobotRounded > 4) {
            climb = true;
        }

        if (climb == true && angleRobotRounded > -3 && angleRobotRounded < 3){
            stop = true;
        }
        
        if (climb == false) {
            motoresTank.SetTankDrive(-0.5, -0.5);
        }
        else if (angleRobotRounded < 10 && stop == false) {
            motoresTank.SetTankDrive(-0.4, -0.4);
        }
        else if (angleRobotRounded >= 10 && stop == false) {
            motoresTank.SetTankDrive(-0.35, -0.35);
        }
        else if (stop == true) {
            if (angleRobotRounded > 4) {  // 4
                motoresTank.SetTankDrive(-0.25, -0.25);
            }
            else if (angleRobotRounded < -4){ // -4
                motoresTank.SetTankDrive(0.3, 0.3);  // 0.25
            }
            else {
                motoresTank.SetTankDrive(0, 0);
            }
        }
    }
}
