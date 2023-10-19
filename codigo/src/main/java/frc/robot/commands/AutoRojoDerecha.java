package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
import frc.robot.subsystems.Hombro;
import frc.robot.Constants;
import frc.robot.subsystems.Garra;
import frc.robot.subsystems.Brazo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoRojoDerecha extends CommandBase{

    private DriveTrainSubsystemModified driveTrain;
    private Hombro hombro;
    private Garra garra;
    private Brazo brazo;

    
    
    public AutoRojoDerecha(DriveTrainSubsystemModified driveTrain, Hombro hombro, Garra garra, Brazo brazo){
        this.driveTrain = driveTrain;
        this.hombro = hombro;
        this.garra = garra;
        this.brazo = brazo;

        addRequirements(driveTrain);
        addRequirements(hombro);
        addRequirements(garra);
        addRequirements(brazo);
    }

    private static double reqHombroPosition;
    private static double reqWristPosition;
    private static double reqArmPosition;
    private static boolean done;
    private static boolean flag;
    
    private static double xObjective;
    private static double yObjective;
    private static double zObjective;

    private static int autoStage;

    private static double kP1x;
    private static double kP1y;
    private static double kP1z;
    private static double kP2x;
    private static double kP2y;
    private static double kP2z;


    @Override
    public void initialize() {
  
      driveTrain.resetNavx();

      //driveTrain.setOdoToLimelight();
      System.out.println("Autos moda y rock&roll command started");
      xObjective = driveTrain.getPose2d().getX();
      zObjective = Constants.pi/2;
      yObjective = -1;
      autoStage = 0;
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

      if(autoStage == 0){

        if(Math.abs(ySpeed) < 0.2){
        autoStage++;
        driveTrain.setOdoToLimelight();
        xObjective = 10.8;
        yObjective = 4.8;
        zObjective = Constants.pi/2;
        Timer.delay(0.5);
      }
      }
      else if(autoStage == 1){
        
        if(Math.abs(xSpeed) < 0.2){
          autoStage++;
          Timer.delay(0.5);
          xObjective = driveTrain.getPose2d().getX();
          yObjective = driveTrain.getPose2d().getY(); 
          zObjective = 3 * Constants.pi / 4;
          kP1z = -1.5;
        }
      }
      else if(autoStage == 2){
        
        if(Math.abs(zSpeed) < 0.05){
          autoStage++;
          Timer.delay(0.5);
          xObjective = 10.1;
          yObjective = 5.3;
          zObjective = driveTrain.getNavxYawRadians();
          kP1x = -0.5;
          kP1y = -0.5;
        }
      }
      else if(autoStage == 3){
        
        if(Math.abs(xSpeed) < 0.2 && Math.abs(ySpeed) < 0.2){
          autoStage++;
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
