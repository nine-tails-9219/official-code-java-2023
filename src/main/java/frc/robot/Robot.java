// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//#region Bibliotecas

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//#endregion

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
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Controles
  private XboxController xboxControllerTank = new XboxController(0);
  private XboxController xboxControllerTurrent = new XboxController(1);

  // Motores
  private MotoresTank motoresTank;
  private MotoresTurrent motoresTurrent;

  // Autonomo
  private PeriodoAutonomo autonomoRobot;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    motoresTank = new MotoresTank(); // Iniciar os motores
    motoresTurrent = new MotoresTurrent();
    autonomoRobot = new PeriodoAutonomo(motoresTank); // Iniciar autonomo
  }

  @Override
  public void robotPeriodic() {
    
    // Att valores Piegon 2.0
    autonomoRobot.UpdatePigeon();

    // Dashboard dos motores 
    motoresTank.UpdateDashboardInfo();

    // Dashboard Autonomo
    autonomoRobot.UpdateDashboardInfo();
  }

  //#region Structure Autonomous

  @Override
  public void autonomousInit() {

    // Seta o Pigeon para o ângulo zero
    autonomoRobot.ResetPigeon();

    // Zera o timer
    autonomoRobot.ResetTimer();
    motoresTank.SetMaxOutput(1.0);
    
    // Zera as variáveis usadas no autônomo
    autonomoRobot.ResetInterruptors();

    // Posiciona o encoder do motor esquerdo para 0
    motoresTank.SetPositionEncoderMotorLeft(0);
  }

  @Override
  public void autonomousPeriodic() {
    autonomoRobot.PeriodoAutonomo();
  }

  //#region Structure Teleop

  @Override
  public void teleopInit() {
    motoresTank.SetSafetyEnabled(false);
  }

  @Override
  public void teleopPeriodic() {
    /*motoresTank.TankController(// Controla a movimentação do robô
      xboxControllerTank.getAButton(),
      xboxControllerTank.getLeftY(), 
      xboxControllerTank.getLeftX(), 
      xboxControllerTank.getLeftTriggerAxis(), 
      xboxControllerTank.getRightTriggerAxis()
      );*/
    motoresTurrent.TurrentController(xboxControllerTurrent.getLeftY(),xboxControllerTurrent.getLeftX(),xboxControllerTurrent.getRightY());
  }

  //#endregion

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