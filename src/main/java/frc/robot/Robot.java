// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//#region Bibliotecas

package frc.robot;
import java.util.Map;
import java.util.concurrent.CancellationException;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//#endregion

/*
  ID Controles:
    • Movimento de Tank: 0
    • Movimento do Intake: 1

  ID PDP: 0

  IDs de Tank:
    • 1 e 3: Direita
    • 2 e 4: Esquerda e Invertido

  IDs de Intake:
    • 5: Angulação do braço (movimento angular)
    • 6: Estender Horizontal (movimento horizontal)
    • 7: Elevação Vertical

  ID PneumaticHUB: 8
*/

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //#region Definindo variáveis de controles, motores, pneumática e câmera

  // Controles
  private XboxController xboxControllerTank = new XboxController(0);
  private XboxController xboxControllerAttachments = new XboxController(1);

  // Motores
  private DifferentialDrive mydrive;
  private Motor motores;
  private final int IDMOTOR1 = 1, IDMOTOR2 = 2, IDMOTOR3 = 3, IDMOTOR4 = 4, IDMOTOR5 = 5, IDMOTOR6 = 6, IDMOTOR7 = 7,IDPNEUMATICHUB = 8, IDPIGEON = 9;
  private CANSparkMax motorArmController = new CANSparkMax(IDMOTOR5, MotorType.kBrushless);
  private WPI_VictorSPX motorExtendArm = new WPI_VictorSPX(IDMOTOR6);
  private WPI_VictorSPX motorElevation = new WPI_VictorSPX(IDMOTOR7);

  // Encoder(s)
  private double valueEncoderMotorArmController = 0;

  // Pneumática

  private final Compressor compressor = new Compressor(IDPNEUMATICHUB, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(IDPNEUMATICHUB, PneumaticsModuleType.REVPH, 0, 2);

  // Pigeon 2.0
  private Pigeon2 pigeon2 = new Pigeon2(IDPIGEON);
  private double angleRobot = 0;
  private double angleRobotRounded = 0;
  private boolean climb = false;

  // Timer
  private final Timer esperaTimer = new Timer();

  //#endregion

  //#region Robot

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // UsbCamera armCamera = CameraServer.startAutomaticCapture(0);
    // UsbCamera tankCamera = CameraServer.startAutomaticCapture(1);
    // armCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);
    // tankCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 30);

    motores = new Motor(IDMOTOR2,IDMOTOR4,IDMOTOR1,IDMOTOR3); // Iniciar os motores
    mydrive = new DifferentialDrive(motores.GetMotorLeft(), motores.GetMotorRight()); // Define o direcionador
    // compressor.enableDigital();  // Ativa o compressor
    compressor.disable();

    doubleSolenoid.set(Value.kOff);

    mydrive.setSafetyEnabled(false);
    climb = false;

    motorArmController.setInverted(true);

    motorArmController.getEncoder().setPosition(0);
    valueEncoderMotorArmController = 0;
  }

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("encoder value", motores.leftEncoder2.getPosition());
    // SmartDashboard.putNumber("encoder left", motores.GetLeftEncoder().getPosition());
    // SmartDashboard.putNumber("encoder right", motores.GetRightEncoder().getPosition());

    SmartDashboard.putNumber("Distância Rodada", GetSensorPosition());
    SmartDashboard.putNumber("Encoder Arm", valueEncoderMotorArmController);
    SmartDashboard.putNumber("Roll", pigeon2.getRoll()); 
  
    SmartDashboard.putBoolean("Subiu", climb);
    SmartDashboard.putBoolean("Parar", stop);
    SmartDashboard.putNumber("Ângulo", angleRobotRounded);
    angleRobot = (pigeon2.getRoll() * -1) - 5;
    // angleRobotRounded = Math.round(angleRobot * 100) / 100;
    angleRobotRounded = (angleRobot * 100) / 100;

    SmartDashboard.putNumber("Timer", esperaTimer.get());

    SmartDashboard.putNumber("Velocidade", motores.leftEncoder1.getVelocity());
  }

  //#endregion

  //#region Parte autonôma

  @Override
  public void autonomousInit() {

    /*m_timer.restart();
    motores.GetLeftEncoder().setPosition(0.0);
    motores.GetRightEncoder().setPosition(0.0);
    mydrive.setMaxOutput(1);*/

    esperaTimer.restart();

    mydrive.setMaxOutput(1.0);
    pigeon2.configFactoryDefault();

    climb = false;
    stop = false;
  }

  final double kP = 0.005;
  double setPoint = 100;
  boolean stop = false;
  
  @Override
  public void autonomousPeriodic() {    
    
    if (angleRobotRounded > 4) {
      climb = true;
    }

    if (climb == true && angleRobotRounded > -3 && angleRobotRounded < 3){
      stop = true;
    }

    if (climb == false) {
      mydrive.tankDrive(-0.4, -0.4);
    }
    else if (angleRobotRounded < 10 && stop == false) {
      mydrive.tankDrive(-0.35, -0.35);
    }
    else if (angleRobotRounded >= 10 && stop == false) {
      mydrive.tankDrive(-0.35, -0.35);
    }
    else if (stop == true) {
      if (angleRobotRounded > 4.3) {  // 4
        mydrive.tankDrive(-0.25, -0.25);
      }
      else if (angleRobotRounded < -4.3){ // -4
        mydrive.tankDrive(0.3, 0.3);  // 0.25
      }
      else {
        mydrive.tankDrive(0, 0);
      }
    }
  }

    /* 
    if (motores.GetLeftEncoder().getPosition() < 10.5 && motores.GetRightEncoder().getPosition() < 10.5) {
      mydrive.tankDrive(-0.3, -0.3);
    }
    else {
      mydrive.stopMotor();
      mydrive.tankDrive(0, 0);
    }
    */

  //#endregion
  
  //#region Parte Teleoperada

  @Override
  public void teleopInit() {
    mydrive.setSafetyEnabled(false);
    //motorArmController.getEncoder().setPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    TankController(); // Controla a movimentação do robô
    ControlBody(); // Controla a elevação vertical, o movimento do braço  e o de coleta

    // SmartDashboard.putNumber("LeftMotor:", motores.GetLeftEncoder().get());
    // SmartDashboard.putNumber("RightMotor:", motores.GetRightEncoder().get());
  }

  //#endregion

  //#region TankControlller
  
  private void TankController() {
    SetVelocityMode();
    MovimentationTank();
  }

  private void MovimentationTank(){
    if (xboxControllerTank.getAButton()) {
      mydrive.stopMotor();
    }
    else if (Math.abs(xboxControllerTank.getLeftY()) >= 0.05 || Math.abs(xboxControllerTank.getLeftX()) >= 0.05) {  // Movendo Joystick
      mydrive.arcadeDrive(xboxControllerTank.getLeftY(), xboxControllerTank.getLeftX()*1.2);
    }
    else {
      mydrive.stopMotor();
    }
  }

  private void SetVelocityMode(){
    if (xboxControllerTank.getLeftTriggerAxis() > 0) {
      mydrive.setMaxOutput(0.3);
    }
    else if (xboxControllerTank.getRightTriggerAxis() > 0) {
      mydrive.setMaxOutput(1);
    }
    else {
      mydrive.setMaxOutput(0.5);
    }
  }

  private Double GetSensorPosition() {
    return ((motores.GetLeftEncoder().getPosition() * 2 * Math.PI * 7.6) / motores.GetLeftEncoder().getCountsPerRevolution()) * 400;
  }

  //#endregion
 
  //#region ControlBody

  private void ControlBody() {
    ControlElevation();
    ExtendArm();
    // ControlArm();
    if(xboxControllerAttachments.getLeftY()> 0.1 && xboxControllerAttachments.getLeftY()< -0.1){
        motorArmController.set(xboxControllerAttachments.getLeftY()*0.85);
      }
    else{
        motorArmController.stopMotor();
    }
    // valueEncoderMotorArmController = motorArmController.getEncoder().getPosition();
    ControlIntake();
  }

  
  private void ControlElevation() {  
    // Apenas um poderá ser acionado por vez
    if ((xboxControllerAttachments.getLeftTriggerAxis() > 0 && xboxControllerAttachments.getRightTriggerAxis() == 0) || (xboxControllerAttachments.getLeftTriggerAxis() == 0 && xboxControllerAttachments.getRightTriggerAxis() > 0)) {
      if (xboxControllerAttachments.getLeftTriggerAxis() > 0) { // Descer elevador
        motorElevation.set(xboxControllerAttachments.getLeftTriggerAxis());
      }
      else if (xboxControllerAttachments.getRightTriggerAxis() > 0) { // Subir elevador
        motorElevation.set(xboxControllerAttachments.getRightTriggerAxis() * -1);
      }
    }
    else {
      motorElevation.stopMotor();
    }
  }

  private void ExtendArm() {
    if (xboxControllerAttachments.getRightY() != 0) {
      motorExtendArm.set(ControlMode.PercentOutput, xboxControllerAttachments.getRightY());
    }
    else {
      motorExtendArm.stopMotor();
    }
  }
  

  private void ControlArm() {
    valueEncoderMotorArmController = motorArmController.getEncoder().getPosition();
    if(xboxControllerAttachments.getLeftY()>0){
      if(valueEncoderMotorArmController>0){
        motorArmController.set(xboxControllerAttachments.getLeftY()*0.85);
      }
      else{
        motorArmController.stopMotor();
      }
    }
    else if(xboxControllerAttachments.getLeftY()<0){
      if(valueEncoderMotorArmController<153){
        motorArmController.set(xboxControllerAttachments.getLeftY()*0.85);
      }
      else{
        motorArmController.stopMotor();
      }
    }
    else{
      motorArmController.stopMotor();
    }

    /*valueEncoderMotorArmController = motorArmController.getEncoder().getPosition();
    if (xboxControllerAttachments.getLeftY() != 0 ) {
      motorArmController.set(xboxControllerAttachments.getLeftY()*0.2);
    }
    else {
      motorArmController.stopMotor();
    }*/

    /* 
    if(valueEncoderMotorArmController<=0){
      if (xboxControllerAttachments.getLeftY() <= 0 ) {
        motorArmController.set(xboxControllerAttachments.getLeftY());
      }
      else {
        motorArmController.stopMotor();
      }
    }
    else if(valueEncoderMotorArmController>0) {
      if (xboxControllerAttachments.getLeftY() >= 0 ) {
        motorArmController.set(xboxControllerAttachments.getLeftY());
      }
      else {
        motorArmController.stopMotor();
      }
    }
    else{
      motorArmController.stopMotor();
    }*/
  }

  private void ControlIntake() {
    if (xboxControllerAttachments.getLeftBumper()) {  // Abrir
      doubleSolenoid.set(Value.kReverse);
    }
    else if (xboxControllerAttachments.getRightBumper()) {  // Fecahr
      doubleSolenoid.set(Value.kForward);
    }
  }
  

  //#endregion

  //#region Metodos Off

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  //#endregion

}