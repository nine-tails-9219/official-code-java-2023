package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class MotoresTurrent {
    public CANSparkMax leftMotor1;
    public CANSparkMax leftMotor2;
    public CANSparkMax rightMotor1;
    public CANSparkMax rightMotor2;

    public RelativeEncoder leftEncoder1;
    private RelativeEncoder leftEncoder2;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    private VictorSPX motorRail;

    private DifferentialDrive controladorMotores;

    private final int ID_MOTOR_LETF1 = 5;
    private final int ID_MOTOR_LEFT2 = 7;
    private final int ID_MOTOR_RIGHT1 = 6;
    private final int ID_MOTOR_RIGHT2 = 8;
    private final int ID_MOTOR_TRILHO = 9;

    private double speed = 0.5f;
    private double speedRail = 0.5f;

    public MotoresTurrent(){
        // Definindo os dois motores da esquerda e os dois da direita
        leftMotor1 = new CANSparkMax(ID_MOTOR_LETF1, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(ID_MOTOR_LEFT2, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(ID_MOTOR_RIGHT1, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(ID_MOTOR_RIGHT2, MotorType.kBrushless);

        motorRail = new VictorSPX(ID_MOTOR_TRILHO);
        motorRail.set(ControlMode.PercentOutput,0);

        leftEncoder1 = leftMotor1.getEncoder();
        leftEncoder2 = leftMotor2.getEncoder();

        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

        controladorMotores = new DifferentialDrive(leftMotors, rightMotors); // Define o direcionador
        SetSafetyEnabled(false);
    }

    public void TurrentController(double leftY, double leftX, double rightY){
        MovimentationTurrent(leftY, leftX);
        MovimentationRailTurrent(rightY);
    }

    // Controle da torreta do robo
    private void MovimentationTurrent(double leftY, double leftX){
        if (Math.abs(leftY) >= 0.05) {  // Movendo Joystick
            controladorMotores.arcadeDrive(leftY*speed,0);
        }
        else {
            controladorMotores.stopMotor();
        }
    }

    // Controle do trilho do robo
    private void MovimentationRailTurrent(double rightY){
        if (Math.abs(rightY) >= 0.05) {  // Movendo Joystick
            motorRail.set(ControlMode.PercentOutput, rightY*speedRail);
        }
        else {
            motorRail.set(ControlMode.PercentOutput,0);
        }
    }

    public void SetSafetyEnabled(boolean valor){
        controladorMotores.setSafetyEnabled(valor); // Desativa o travamento do motor por segurança (estava causando erro)
    }
}
