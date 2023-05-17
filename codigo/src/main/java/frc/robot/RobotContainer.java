// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommandModified;

import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystemModified m_DriveTrainSubsystemModified = new DriveTrainSubsystemModified();
 
  private final XboxController driveController = new XboxController(Constants.driveControllerID);

 
  public RobotContainer() {
    
    m_DriveTrainSubsystemModified.setDefaultCommand(new DriveCommandModified(m_DriveTrainSubsystemModified, () -> driveController.getLeftX(),
    () -> driveController.getLeftY(), () -> driveController.getRightX(), () -> driveController.getBackButtonPressed()));
   
   

    configureBindings();
  }

  private void configureBindings() {
    
   
    

    
    System.out.println("Corriendo \\(-v-)/");
  }


  public Command getAutonomousCommand() {



    return null;
  }
}
