package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoRojoDerecha extends CommandBase{

    private DriveTrainSubsystemModified driveTrain;

    
    
    public AutoRojoDerecha(DriveTrainSubsystemModified driveTrain){
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);
    }

    
    private static double xObjective;
    private static double yObjective;
    private static double zObjective;

    private static int autoStage;

    private static double kP1x;
    private static double kP1y;
    private static double kP1z;


    @Override
    public void initialize() {
  
      driveTrain.resetNavx();

      //driveTrain.setOdoToLimelight();
      System.out.println("Autos moda y rock&roll command started");
      xObjective = driveTrain.getPose2d().getX();
      zObjective = Constants.pi/2;
      yObjective = driveTrain.getPose2d().getY();
      autoStage = -1;
      kP1x = -0.8;
      kP1y = -0.8;
      kP1z = -2;
    }

    

    @Override
    public void execute(){
      double yaw = driveTrain.getNavxYawRadians();
      if(yaw > Constants.pi){
        zObjective += 2*Constants.pi;
      }
      double zSpeed = (yaw - zObjective)*kP1z;

      if(zSpeed > 1){
        zSpeed = 1;
      } else if (zSpeed < -1){
        zSpeed = -1;
      }

      double xSpeed = (driveTrain.getPose2d().getX() - xObjective) * kP1x;

      if(xSpeed > 0.5){
        xSpeed = 0.5;
      } else if (xSpeed < -0.5){
        xSpeed = -0.5;
      }

      double ySpeed = (driveTrain.getPose2d().getY() - yObjective) * kP1y;

      if(ySpeed > 0.5){
        ySpeed = 0.5;
      } else if (ySpeed < -0.5){
        ySpeed = -0.5;
      }

      SmartDashboard.putNumber("yaw", yaw);
      SmartDashboard.putNumber("zSpeed", zSpeed);

      driveTrain.setFieldOrientedSpeeds(ySpeed, xSpeed, zSpeed);

      if(autoStage == -1){

        xObjective = driveTrain.getPose2d().getX();
        zObjective = Constants.pi/2;
        yObjective = -1;
        autoStage++;
      }
      else if(autoStage == 0){

        if(Math.abs(xSpeed) < 0.2 && Math.abs(ySpeed) < 0.2){
        autoStage++;
        driveTrain.setOdoToLimelight();
        xObjective = 10.8;
        yObjective = 4.6;
        zObjective = Constants.pi/2;
      }
      }
      else if(autoStage == 1){
        
        
        if(Math.abs(xSpeed) < 0.2 && Math.abs(ySpeed) < 0.2){
          autoStage++;
          xObjective = 13;
          yObjective = 4.3; 
        }
      }
      else if(autoStage == 2){
        
      
        if(Math.abs(xSpeed) < 0.2 && Math.abs(ySpeed) < 0.2){
          autoStage++;
          driveTrain.setOdoToLimelight();
          xObjective = 15.6;
          yObjective = 4.5; 
        }
      }
      else{

        
        driveTrain.stopModules();
      }
      SmartDashboard.putNumber("auto stage", autoStage);
    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto derecha rojo command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    return false;
  }



}
