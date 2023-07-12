// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//#endregion

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Array;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
  ID Controles:
    • Movimento de Tank: 0
    • Movimento do Intake: 1

  ID PDP: 0

  IDs de Tank:
    • 1 e 3: Direita
    • 2 e 4: Esquerda e Invertido

  IDs de Body:
    • 5: Angulação do braço (movimento angular)
    • 6: Estender Horizontal (movimento horizontal)
    • 7: Elevação Vertical

  ID PneumaticHUB: 
    • 8

  ID Pigeon 2.0: 
    • 9
*/

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private static final String kDefaultTeam = "Default Team";
  private static final String kCustomRedAlliance = "Red Alliance";
  private static final String kCustomBlueAlliance = "Blue Alliance";

  private static final String kDefaultPosition = "Default Position";
  private static final String kCustomPosition1 = "Position 1";
  private static final String kCustomPosition2 = "Position 2";
  private static final String kCustomPosition3 = "Position 3";

  private String m_autoSelected;
  private String color_team;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_team_color = new SendableChooser<>();
  private final SendableChooser<String> m_team_position = new SendableChooser<>();
  // private final SendableChooser<String> m_pipeline = new SendableChooser<>();


  //#region Definindo variáveis de controles, motores, encoders, pneumática, câmera, etc.

  // Controles
  private XboxController xboxControllerTank = new XboxController(0);
  private XboxController xboxControllerArm = new XboxController(1);


  // Motores
  private DifferentialDrive mydrive;
  private Motor motores;
  private final int IDMOTOR1 = 1, IDMOTOR2 = 2, IDMOTOR3 = 3, IDMOTOR4 = 4;


  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");

  // Pigeon2 pigeon = new Pigeon2(9);


  // private final int IDMOTOR5 = 5;
  // public CANSparkMax motorTeste;

  PIDController pidChargeStation  = new PIDController(0.048, 0, 0);
  PIDController pidFollow  = new PIDController(0.2, 0, 0);
  PIDController pidTurn  = new PIDController(0.02, 0, 0.0); //0.065, 0.0023, 0.01
  PIDController pidLimelight = new PIDController(IDMOTOR2, IDMOTOR1, kDefaultPeriod);

  // Pigeon 2.0
  private Pigeon2 pigeon2 = new Pigeon2(IDPIGEON);
  private double angleRobot = 0;
  private double angleRobotRounded = 0;

  // Timer
  private final Timer esperaTimer = new Timer();

  // Interruptores
  private boolean climb = false;
  private boolean stop = false;
  private boolean stopSolenoid = false;
  private boolean conditionIdAutonomo3 = false;

  // ID Autonomo
  private int idAutonomo = 0;

  //#endregion

  //#region Robot

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_team_color.setDefaultOption("Default Color", kDefaultTeam);
    m_team_color.addOption("Red Alliance", kCustomRedAlliance);
    m_team_color.addOption("Blue Alliance", kCustomBlueAlliance);
    SmartDashboard.putData("Team Color", m_team_color);

    m_team_position.setDefaultOption("Default Position", kDefaultPosition);
    m_team_position.addOption("Position 1", kCustomPosition1);
    m_team_position.addOption("Position 2", kCustomPosition2);
    m_team_position.addOption("Position 3", kCustomPosition3);
    SmartDashboard.putData("Team Position", m_team_position);

    // m_pipeline.setDefaultOption("Pipeline 0", "0");
    // m_pipeline.addOption("Pipeline 1", "1");
    // m_pipeline.addOption("Pipeline 2", "2");
    // m_pipeline.addOption("Pipeline 3", "3");
    // m_pipeline.addOption("Pipeline 4", "4");
    // SmartDashboard.putData("Pipeline", m_pipeline);

    motores = new Motor(IDMOTOR2,IDMOTOR4,IDMOTOR1,IDMOTOR3); // Iniciar os motores
    mydrive = new DifferentialDrive(motores.GetMotorLeft(), motores.GetMotorRight()); // Define o direcionador
    mydrive.setSafetyEnabled(false);

    motorArmController.setInverted(true); // Inverte o sentido do motor do cabo de aço

    // motorTeste = new CANSparkMax(IDMOTOR5, MotorType.kBrushless);

    //#region Resetar PID

    pidChargeStation.reset();
    pidFollow.reset();
    pidTurn.reset();

    //#endregion
  }

  @Override
  public void robotPeriodic() {
    //#region Pigeon

    /* SmartDashboard.putNumber("Pigeon (Yaw)", pigeon.getYaw());
    SmartDashboard.putNumber("Pigeon (Pitch)", pigeon.getPitch());
    SmartDashboard.putNumber("Pigeon (Roll)", pigeon.getRoll()*-1);
    */

    //#endregion
    //#region Limelight

    SmartDashboard.putNumber("LL3 (tx)", tx.getDouble(0));
    SmartDashboard.putNumber("LL3 (ty)", ty.getDouble(0));
    SmartDashboard.putNumber("LL3 (ta)", ta.getDouble(0));
    SmartDashboard.putNumber("LL3 (ID)", tid.getDouble(0));

    //#endregion
    // table.getEntry("pipeline").setNumber(Integer.parseInt(m_pipeline.getSelected()));
    //ResetYaw();
  }
 
  /* 
  private void ResetYaw() {
    if (Math.abs(pigeon.getYaw()) >= 360 || xboxControllerTank.getRightBumper()) {
      pigeon.setYaw(0);
    }
  }
  */

  @Override
  public void autonomousInit() {
    pigeon2.setYaw(0);
    pigeon2.configFactoryDefault();

    esperaTimer.restart();
    mydrive.setMaxOutput(1.0);
    
    climb = false;
    stop = false;
    stopSolenoid = false;
    conditionIdAutonomo3 = false;

    doubleSolenoid.set(Value.kForward); // Fecha a solenoide por padrão

    motores.leftEncoder1.setPosition(0);
    motorArmController.getEncoder().setPosition(0);

    /*==================================
    *   MUDE O PERÍODO AUTÔNOMO AQUI   *
    *    !!!!!!!!!!!!!!!!!!!!!!!!!     *
    ===================================*/
    idAutonomo = 1;
  }

  @Override
  public void autonomousPeriodic() {
    // Criar período autônomo
    mydrive.stopMotor();
  }

  //#endregion
  
  //#region Autonomous methods

  private void DeliverMiddleAndExitCommunity(){
    
    if (valueEncoderMotorArmController < 78 && stopSolenoid == false){    // 78
      motorArmController.set(1);
    }
    else if (valueEncoderMotorArmController >= 78 && stopSolenoid == false){    // 78
      motorArmController.set(0);

      if (stopSolenoid == false) {
        doubleSolenoid.set(Value.kReverse);

        esperaTimer.restart();
        while (esperaTimer.get() < 0.5){
          motorArmController.set(-0.1);
        }

        stopSolenoid = true;
      }
    }
    else if (stopSolenoid == true && valueEncoderMotorArmController > 10) {
        motorArmController.set(-1);  
    }
    else{
      motorArmController.stopMotor();

      // Exit Community
      if (motores.leftEncoder1.getPosition() > -79.5) { //3,5M //Conta = ((QtdEmCmQueDeseja / 47,1) * -10,71)
        mydrive.tankDrive(0.5, 0.5);
      }
      else {
        mydrive.stopMotor();
      }
    }
  }

  private void CubeInLowAndChargeStation(){

    while (esperaTimer.get() < 1.5) {
      mydrive.tankDrive(0.3, 0.3);
    }

    while (esperaTimer.get() < 1.7) {
      mydrive.tankDrive(-0.6, -0.6);
    }

    if (angleRobotRounded > 4) {
      climb = true;
    }

    if (climb == true && angleRobotRounded > -3 && angleRobotRounded < 3){
      stop = true;
    }
    
    if (climb == false) {
      mydrive.tankDrive(-0.5, -0.5);
    }
    else if (angleRobotRounded < 10 && stop == false) {
      mydrive.tankDrive(-0.4, -0.4);
    }
    else if (angleRobotRounded >= 10 && stop == false) {
      mydrive.tankDrive(-0.35, -0.35);
    }
    else if (stop == true) {
      if (angleRobotRounded > 4) {  // 4
        mydrive.tankDrive(-0.25, -0.25);
      }
      else if (angleRobotRounded < -4){ // -4
        mydrive.tankDrive(0.3, 0.3);  // 0.25
      }
      else {
        mydrive.tankDrive(0, 0);
      }
    }
  }

  private void DeliverMiddleAndReverseAnd180AndChargeStation() {

    if (valueEncoderMotorArmController < 78 && stopSolenoid == false){    // 78
      motorArmController.set(1);
    }
    else if (valueEncoderMotorArmController >= 78 && stopSolenoid == false){    // 78
      motorArmController.set(0);

      if (stopSolenoid == false) {
        doubleSolenoid.set(Value.kReverse);

        esperaTimer.restart();
        while (esperaTimer.get() < 0.5){
          motorArmController.set(-0.1);
        }

        stopSolenoid = true;
      }
    }
    else if (stopSolenoid == true && valueEncoderMotorArmController > 10) {
        motorArmController.set(-1);  
    }
    else{
      motorArmController.stopMotor();

      if (conditionIdAutonomo3 == false){
        // ReverseAnd180
        while (motores.GetLeftEncoder().getPosition() > -7) {
          mydrive.tankDrive(0.5, 0.5);
        }

        while (pigeon2.getYaw() < 160){
          mydrive.tankDrive(0.5, -0.5);
        }

        conditionIdAutonomo3 = true;
      }
      else{
        // ChargeStation
        if (angleRobotRounded > 4) {
          climb = true;
        }

        if (climb == true && angleRobotRounded > -3 && angleRobotRounded < 3){
          stop = true;
        }
        
        if (climb == false) {
          mydrive.tankDrive(-0.5, -0.5);
        }
        else if (angleRobotRounded < 10 && stop == false) {
          mydrive.tankDrive(-0.4, -0.4);
        }
        else if (angleRobotRounded >= 10 && stop == false) {
          mydrive.tankDrive(-0.35, -0.35);
        }
        else if (stop == true) {
          if (angleRobotRounded > 4) {  // 4
            mydrive.tankDrive(-0.25, -0.25);
          }
          else if (angleRobotRounded < -4){ // -4
            mydrive.tankDrive(0.3, 0.3);  // 0.25
          }
          else {
            mydrive.tankDrive(0, 0);
          }
        }
      }
    }  
  }
  
  //#endregion

  //#region Structure Teleop

  @Override
  public void teleopInit() {
    mydrive.setSafetyEnabled(false);
    // pigeon.setYaw(0);
    // pigeon.configFactoryDefault();
  }

  @Override
  public void teleopPeriodic() {
    TankController(); // Controla a movimentação do robô
    // LimelightController(); // xButton || yButton
    // ChargeStation();  // leftBumper
    // Turn(90); // Virar || leftStickButton
    // FollowAngle(0.5, 0); // seguir de acordo com o angulo || bButton



  }

  private void Teste() {
    // motorTeste.set(xboxControllerTank.getLeftY()*0.5);
  }

  //#region PID e Teste autônomo
  private void ChargeStation() {
    if (xboxControllerTank.getLeftBumper()){
      // mydrive.setMaxOutput(0.8);
      //double speed = pidChargeStation.calculate(pigeon.getRoll(), -3);
      // mydrive.arcadeDrive(-speed, 0);

  /* PID antigo
      final double kP = 0.048;
      final double setPoint = -5;  // Posição do X desejada
  
      double angle = pigeon.getRoll();
  
      double error = setPoint - angle;
      double speed = kP * error;
  
      mydrive.setMaxOutput(0.8);
      mydrive.arcadeDrive(-speed, 0); */
    }
  }

  /* 
  private void Turn(double angle) {
    if (xboxControllerTank.getLeftStickButton()) {
      double direction = pidTurn.calculate(pigeon.getYaw(), angle);

      mydrive.setMaxOutput(0.4);
      mydrive.arcadeDrive(0, -direction);
    }
  }

  */
  /* 
  private void PIDVirar (double angulo) {
    if (xboxControllerTank.getLeftStickButton()) {
      final double kP = 0.065; // 0.03  0.055
      final double kI = 0.0023; //0.002
      final double kD = 0.01; // 0.01

      final double iLimit = 1;

      final double setPoint = angulo;  // Posição do X desejada
  
      double angle = pigeon.getYaw();
  
      double error = setPoint - angle;
      double dt = Timer.getFPGATimestamp() - lastTimeStamp;

      if (Math.abs(error) < iLimit) {
        errorSum += error + dt;
      }

      double errorRate = (error - lastError) / dt;

      double outputDirection = kP * error + kI * errorSum + kD * errorRate;
  
      mydrive.setMaxOutput(0.4);
      mydrive.arcadeDrive(0, -outputDirection);

      lastTimeStamp = Timer.getFPGATimestamp();
      lastError = error;
    }
  }
  */

  /* 
  private void FollowAngle(double speed, double angle) {
    if (xboxControllerTank.getBButton()){
      double direction = pidFollow.calculate(pigeon.getYaw(), angle);
      mydrive.setMaxOutput(0.3);
      mydrive.arcadeDrive(-speed, -direction);
    }
  }
  */

  /*private void PIDLine() {
    if (xboxControllerTank.getBButton()){
      final double kP = 0.3;
      final double setPoint = 0;  // Posição do X desejada
  
      double angle = pigeon.getYaw();
  
      double error = setPoint - angle;
      double outputDirection = kP * error;
  
      mydrive.setMaxOutput(0.3);
      mydrive.arcadeDrive(xboxControllerTank.getLeftY(), -outputDirection);
    }
  }
  */
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
      mydrive.arcadeDrive(xboxControllerTank.getLeftY(), xboxControllerTank.getLeftX());
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
      mydrive.setMaxOutput(xboxControllerTank.getRightTriggerAxis()+ 0.5);
    }
    else {
      mydrive.setMaxOutput(0.5);  // 0.5
    }
  }

  private void LimelightController() {
    pidLimelight.setPID(0.06, 0, 0);
    double speed = pidLimelight.calculate(ta.getDouble(0), 11);

    pidLimelight.setPID(0.035, 0, 0);
    double direction = pidLimelight.calculate(tx.getDouble(0), 0);

    if (xboxControllerTank.getYButton()) {
      table.getEntry("pipeline").setNumber(1); //7

      mydrive.setMaxOutput(0.6);
      mydrive.arcadeDrive(-speed, -direction);
    }
    else if (xboxControllerTank.getXButton()) {
      table.getEntry("pipeline").setNumber(0);
      
      mydrive.setMaxOutput(0.6);
      mydrive.arcadeDrive(-speed, -direction);
    }
  }

  private double GetDirection() {
    final double kP = 0.035;
    final double setPoint = 0;  // Posição do X desejada

    double XPosition = tx.getDouble(0.0);

    double error = setPoint - XPosition;
    double outputSpeed = kP * error;

    return outputSpeed;
  }
  

  private double GetSpeed() {
    final double kP = 0.06;    //0.28
    final double setPoint = 11;  // Posição da area desejada    1.8

    double area = ta.getDouble(0.0);

    double error = setPoint - area;
    double outputSpeed = kP * error;

    return outputSpeed;
  }

  //#region Methods Off

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