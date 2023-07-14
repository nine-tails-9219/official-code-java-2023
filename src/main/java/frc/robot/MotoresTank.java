package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class MotoresTank {
    public CANSparkMax leftMotor1;
    public CANSparkMax leftMotor2;
    public CANSparkMax rightMotor1;
    public CANSparkMax rightMotor2;

    public RelativeEncoder leftEncoder1;
    private RelativeEncoder leftEncoder2;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    private DifferentialDrive controladorMotores;

    private final int ID_MOTOR_LETF1 = 1;
    private final int ID_MOTOR_LEFT2 = 2;
    private final int ID_MOTOR_RIGHT1 = 3;
    private final int ID_MOTOR_RIGHT2 = 4;

    public MotoresTank(){
        // Definindo os dois motores da esquerda e os dois da direita
        leftMotor1 = new CANSparkMax(ID_MOTOR_LETF1, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(ID_MOTOR_LEFT2, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(ID_MOTOR_RIGHT1, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(ID_MOTOR_RIGHT2, MotorType.kBrushless);

        leftEncoder1 = leftMotor1.getEncoder();
        leftEncoder2 = leftMotor2.getEncoder();

        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

        InvertedMotorLeft();

        controladorMotores = new DifferentialDrive(leftMotors, rightMotors); // Define o direcionador
        SetSafetyEnabled(false); // Desativa o travamento do motor por segurança (Estava causando erro)

    }

    public void TankController(boolean valorButtonA, double leftY, double leftX, double triggerLeft, double triggerRight) {
        SetVelocityMode(triggerLeft, triggerRight);
        MovimentationTank(valorButtonA,leftY, leftX);
    }

    // Controle no chassi com ArcadeDrive no eixo esquerdo do controle
    private void MovimentationTank(boolean valorButtonA, double leftY, double leftX){
        if (valorButtonA) {
            controladorMotores.stopMotor();
        }
        else if (Math.abs(leftY) >= 0.05 || Math.abs(leftX) >= 0.05) {  // Movendo Joystick
            controladorMotores.arcadeDrive(leftY, leftX*1.2);
        }
        else {
            controladorMotores.stopMotor();
        }
    }

    // Metodo feito para tirar a limitação de velocidade e defini-la pelo tanto pressionado no gatilho direito
    private void SetVelocityMode(double triggerLeft, double triggerRight){
        if (triggerLeft > 0) {
            controladorMotores.setMaxOutput(0.3);
        }
        else if (triggerRight > 0) {
            controladorMotores.setMaxOutput(1);
        }
        else {
            controladorMotores.setMaxOutput(0.5);
        }
    }

    private void InvertedMotorLeft(){
        leftMotors.setInverted(true);
    }

    public MotorControllerGroup GetMotorLeft(){
        return leftMotors;
    }

    public MotorControllerGroup GetMotorRight(){
        return rightMotors;
    }

    public void StopMotors() {
        leftMotors.set(0.05);
        rightMotors.set(0.05);
    }

    public RelativeEncoder GetLeftEncoder() {
        return leftEncoder1;
    }

    public RelativeEncoder GetRightEncoder() {
        return leftEncoder2;
    }

    public void SetMaxOutput(double valor){
        controladorMotores.setMaxOutput(valor);
    }

    public void SetPositionEncoderMotorLeft(double valor){
        leftEncoder1.setPosition(valor);
    }

    public void SetSafetyEnabled(boolean valor){
        controladorMotores.setSafetyEnabled(valor); // Desativa o travamento do motor por segurança (estava causando erro)
    }

    // Metodo para obter o encoder do motor (não está sendo utilizado)
    public Double GetSensorPosition() {
        return ((GetLeftEncoder().getPosition() * 2 * Math.PI * 7.6) / GetLeftEncoder().getCountsPerRevolution()) * 400;
    }

    public void UpdateDashboardInfo(){
        //SmartDashboard.putNumber("Encoder Left", GetLeftEncoder().getPosition());
        //SmartDashboard.putNumber("Encoder Right", GetRightEncoder().getPosition());
    }

    public void SetTankDrive(double leftSpeed, double rightSpeed){
        controladorMotores.tankDrive(leftSpeed, rightSpeed);
    }

}