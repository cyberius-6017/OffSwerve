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
    private static double rollerSpeed;
    private static boolean done;
    private static boolean flag;
    
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


      reqWristPosition =8.3;
      reqArmPosition = 8.7;
      reqHombroPosition = 0.395;
      rollerSpeed = 0;
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

        garra.setRoller(0, false);
        hombro.setPosition(reqHombroPosition);
        garra.setWristPosition(reqWristPosition);
        Timer.delay(0.8);
        brazo.setArmPositon(reqArmPosition);
        Timer.delay(0.5);
        garra.setRoller(-0.5, false);
        Timer.delay(1);


        reqHombroPosition = 0.04;
        reqWristPosition = 0.0;
        reqArmPosition = 0.4;
        rollerSpeed = 0;
        brazo.setArmPositon(reqArmPosition);
        Timer.delay(0.3);
        hombro.setPosition(reqHombroPosition);
        Timer.delay(1.3);
        brazo.setReading(0.4);


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
        Timer.delay(0.5);

        reqWristPosition = 9.75;
        rollerSpeed = -0.5;
        garra.setRoller(rollerSpeed, true);
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
      else if(autoStage == 3){
        


        reqWristPosition = 3.5;
        reqArmPosition = 5.5;
        reqHombroPosition = 0.4;
        
        garra.setRoller(0, true);
        hombro.setPosition(reqHombroPosition);
        garra.setWristPosition(reqWristPosition);
        Timer.delay(0.8);
        brazo.setArmPositon(reqArmPosition);
        Timer.delay(0.5);
        garra.setRoller(0.5, true);
        Timer.delay(1);

        reqHombroPosition = 0.04;
        reqWristPosition = 0.0;
        reqArmPosition = 0.4;
        brazo.setArmPositon(reqArmPosition);
        Timer.delay(0.3);
        hombro.setPosition(reqHombroPosition);
        Timer.delay(1.3);
        brazo.setReading(0.4);

          
      }
      else{

        
        driveTrain.stopModules();
      }

      brazo.setArmPositon(reqArmPosition);
      hombro.setPosition(reqHombroPosition);
      garra.setWristPosition(reqWristPosition);
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
