package frc.robot;

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

    private DifferentialDrive controladorMotores;

    private final int ID_MOTOR_LETF1 = 5;
    private final int ID_MOTOR_LEFT2 = 6;
    private final int ID_MOTOR_RIGHT1 = 7;
    private final int ID_MOTOR_RIGHT2 = 8;

    public MotoresTurrent(){
        // Definindo os dois motores da esquerda e os dois da direita
        leftMotor1 = new CANSparkMax(ID_MOTOR_LETF1, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(ID_MOTOR_LEFT2, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(ID_MOTOR_RIGHT1, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(ID_MOTOR_RIGHT2, MotorType.kBrushless);

        leftEncoder1 = leftMotor1.getEncoder();
        leftEncoder2 = leftMotor2.getEncoder();

        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

        controladorMotores = new DifferentialDrive(leftMotors, rightMotors); // Define o direcionador
        SetSafetyEnabled(false);
    }

    public void TurrentController(){
        
    }

    public void SetSafetyEnabled(boolean valor){
        controladorMotores.setSafetyEnabled(valor); // Desativa o travamento do motor por segurança (estava causando erro)
    }
}
