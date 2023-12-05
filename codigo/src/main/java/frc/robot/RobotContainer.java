// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.BrazoCommand;
import frc.robot.commands.DriveCommandModified;

import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
<<<<<<< Updated upstream
import frc.robot.subsystems.Hombro;
=======
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoRojoDerecha;
import edu.wpi.first.wpilibj.DigitalInput;
>>>>>>> Stashed changes


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
 
public class RobotContainer {
  private final DriveTrainSubsystemModified m_DriveTrainSubsystemModified = new DriveTrainSubsystemModified();
 
  private final XboxController driveController = new XboxController(Constants.driveControllerID);

<<<<<<< Updated upstream
  private final Hombro hombro = new Hombro(Constants.hombroId1, Constants.hombroId2, Constants.hombroId3, Constants.hombroId4);
 
=======

  private final Command autoRojoDerecha = new AutoRojoDerecha(m_DriveTrainSubsystemModified);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

>>>>>>> Stashed changes
  public RobotContainer() {
    
    m_DriveTrainSubsystemModified.setDefaultCommand(new DriveCommandModified(m_DriveTrainSubsystemModified, () -> driveController.getLeftX(),
    () -> driveController.getLeftY(), () -> driveController.getRightX(), () -> driveController.getRightBumperPressed(), () -> driveController.getBButtonPressed()));
   
<<<<<<< Updated upstream
   hombro.setDefaultCommand(new BrazoCommand(hombro, ()-> mechanismController.getLeftY()));
=======

    autoChooser.addOption("Rojo libre", autoRojoDerecha);


    SmartDashboard.putData("autos",autoChooser);
>>>>>>> Stashed changes

    configureBindings();
  }

  private void configureBindings() {
    
   
    

    
    System.out.println("Corriendo \\(-v-)/");
  }


  public Command getAutonomousCommand() {



    return null;
  }
}
