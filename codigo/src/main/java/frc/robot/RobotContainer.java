// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.DriveCommandModified;

import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
import frc.robot.subsystems.Hombro;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Garra;
import frc.robot.commands.BrazoCommand;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
 
public class RobotContainer {
  private final DriveTrainSubsystemModified m_DriveTrainSubsystemModified = new DriveTrainSubsystemModified();
 
  private final XboxController driveController = new XboxController(Constants.driveControllerID);
  private final XboxController mechanismController = new XboxController(Constants.mechanismControllerId);

  private final Garra m_garra = new Garra(Constants.wristId, Constants.rollerId);

  private final Hombro m_hombro = new Hombro(Constants.hombroId1, Constants.hombroId2, Constants.hombroId3, Constants.hombroId4);

  private final Brazo m_brazo = new Brazo(62 , 61);

  private BrazoCommand defaultBrazoCommand =new BrazoCommand(m_hombro, m_garra, m_brazo, ()->  mechanismController.getLeftY(), ()-> mechanismController.getRightY(), ()-> mechanismController.getLeftTriggerAxis(), () -> mechanismController.getRightTriggerAxis(), () -> mechanismController.getAButtonPressed(), () -> mechanismController.getBButtonPressed(), () -> mechanismController.getXButtonPressed(), () -> mechanismController.getYButtonPressed(), ()-> mechanismController.getStartButtonPressed());

  public RobotContainer() {
    
    m_DriveTrainSubsystemModified.setDefaultCommand(new DriveCommandModified(m_DriveTrainSubsystemModified, () -> driveController.getLeftX(),
    () -> driveController.getLeftY(), () -> driveController.getRightX(), () -> driveController.getRightBumperPressed(), () -> driveController.getBButtonPressed()));
   
    m_hombro.setDefaultCommand(defaultBrazoCommand);
    m_garra.setDefaultCommand(defaultBrazoCommand);
    m_brazo.setDefaultCommand(defaultBrazoCommand);


    configureBindings();
  }

  private void configureBindings() {
    
   
    

    
    System.out.println("Corriendo \\(-v-)/");
  }


  public Command getAutonomousCommand() {

   return null;


  }
}
