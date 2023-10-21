// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.DriveCommandModified;

import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hombro;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Garra;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.AutoRojoDerecha;
import frc.robot.commands.BrazoCommand;
import edu.wpi.first.wpilibj.DigitalInput;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
 
public class RobotContainer {
  private final DigitalInput brakeButton = new DigitalInput(0);
  private final DriveTrainSubsystemModified m_DriveTrainSubsystemModified = new DriveTrainSubsystemModified(() -> brakeButton.get());
  
 
  private final XboxController driveController = new XboxController(Constants.driveControllerID);
  private final XboxController mechanismController = new XboxController(Constants.mechanismControllerId);

  private final Garra m_garra = new Garra(Constants.wristId, Constants.rollerId, () -> brakeButton.get());

  private final Hombro m_hombro = new Hombro(Constants.hombroId1, Constants.hombroId2, Constants.hombroId3, Constants.hombroId4, () -> brakeButton.get());

  private final Brazo m_brazo = new Brazo(47 , 48, () -> brakeButton.get());

  private BrazoCommand defaultBrazoCommand =new BrazoCommand(m_hombro, m_garra, m_brazo, ()->  mechanismController.getLeftY(), ()-> mechanismController.getRightY(), ()-> mechanismController.getLeftTriggerAxis(), () -> mechanismController.getRightTriggerAxis(), () -> mechanismController.getAButtonPressed(), () -> mechanismController.getBButtonPressed(), () -> mechanismController.getXButtonPressed(), () -> mechanismController.getYButtonPressed(), ()-> mechanismController.getStartButtonPressed(), ()-> mechanismController.getRightBumperPressed(), () -> mechanismController.getRightStickButtonPressed(), () -> mechanismController.getLeftBumperPressed(), () -> mechanismController.getLeftStickButtonPressed());

  private final Command autoCommand = new AutoCommand(m_DriveTrainSubsystemModified, m_hombro, m_garra, m_brazo);
  private final Command autoRojoDerecha = new AutoRojoDerecha(m_DriveTrainSubsystemModified, m_hombro, m_garra, m_brazo);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    
    m_DriveTrainSubsystemModified.setDefaultCommand(new DriveCommandModified(m_DriveTrainSubsystemModified, () -> driveController.getLeftX(),
    () -> driveController.getLeftY(), () -> driveController.getRightX(), () -> driveController.getRightBumperPressed(), () -> driveController.getBButtonPressed()));
   
    m_hombro.setDefaultCommand(defaultBrazoCommand);
    m_garra.setDefaultCommand(defaultBrazoCommand);
    m_brazo.setDefaultCommand(defaultBrazoCommand);


    autoChooser.addOption("Balance", autoCommand);
    autoChooser.addOption("Rojo libre", autoRojoDerecha);

    SmartDashboard.putData("autos",autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    
   
    

    
    System.out.println("Corriendo \\(-v-)/");
  }


  public Command getAutonomousCommand() {

   return autoChooser.getSelected();


  }
}
