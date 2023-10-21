package frc.robot.commands;

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


public class AutoCommand extends CommandBase{

    private DriveTrainSubsystemModified driveTrain;
    private Hombro hombro;
    private Garra garra;
    private Brazo brazo;

    
    
    public AutoCommand(DriveTrainSubsystemModified driveTrain, Hombro hombro, Garra garra, Brazo brazo){
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
    



    @Override
    public void initialize() {
  
      reqWristPosition =8.3;
      reqArmPosition = 8.7;
      reqHombroPosition = 0.395;
      done = false;
      flag = false;

      brazo.setReading(0);
      garra.setReading(0);
      driveTrain.resetNavx();
      System.out.println("Autos moda y rock&roll command started");
    }

    

    @Override
    public void execute(){
 
      if(!done){

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
      brazo.setArmPositon(reqArmPosition);
      Timer.delay(0.3);
      hombro.setPosition(reqHombroPosition);
      Timer.delay(1.3);
      brazo.setReading(0.4);
      
      flag = false;
      while(!flag){
        driveTrain.setFieldOrientedSpeeds(0,-0.6, 0);
        garra.setRoller(0, false);
        if(driveTrain.getPose2d().getX() < -2.4){
          flag = true;
        }
        driveTrain.updateOdo();
      }
      }
      done = true;
      
      double navxReading = driveTrain.getNavxRollDegrees();
      double ySpeed = navxReading * 0.015;
      driveTrain.setFieldOrientedSpeeds(0, ySpeed, 0);
      SmartDashboard.putNumber("navx reading",navxReading);
      SmartDashboard.putNumber("ySpped", ySpeed);
        
        
      

    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autos moda y rock&roll  command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    return false;
  }



}
