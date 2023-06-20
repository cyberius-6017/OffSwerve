// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;

/** An example command that uses an example subsystem. */
public class DriveCommandModified extends CommandBase {


  private final DriveTrainSubsystemModified driveTrain;
  private final Supplier<Double> xFunction, yFunction, turnFunction;
  private final Supplier<Boolean>  backButton;
  private  boolean pressed = false;
  

   public DriveCommandModified(DriveTrainSubsystemModified driveTrain,  Supplier<Double> xFunction, Supplier<Double> yFunction, Supplier<Double> turnFunction, Supplier<Boolean> backButton)  {
    this.driveTrain = driveTrain;
    this.xFunction = xFunction;
    this.yFunction = yFunction;
    this.turnFunction = turnFunction;
    this.backButton = backButton;

    addRequirements(driveTrain);
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveTrain.resetDriveEncodersPosition();
    driveTrain.resetNavx();

    System.out.println("Modified drive command started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double realXValue = xFunction.get();
    double realYValue = yFunction.get();
    double realTurnValue = turnFunction.get();

    
    driveTrain.setFieldOrientedSpeeds(realXValue, realYValue, realTurnValue);

   if (backButton.get()) { 
     if (pressed == false ){
      pressed = true;
     } else {
       pressed = false;
     }
   } 

   if (pressed == true){
    driveTrain.setNormalSpeeds(realXValue, realYValue, realTurnValue, 0.4);
   } else {
    driveTrain.setFieldOrientedSpeeds(realXValue, realYValue, realTurnValue);
   }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Modified drive command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
