package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
import frc.robot.subsystems.Hombro;
import frc.robot.Constants;


public class AutoCommand extends CommandBase{

    private DriveTrainSubsystemModified driveTrain;

    
    
    public AutoCommand(DriveTrainSubsystemModified driveTrain){
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
  
  
      System.out.println("Autos moda y rock&roll command started");
    }
    static double degrees;
    static boolean arrived = false;;
    @Override
    public void execute(){
      degrees = driveTrain.getNavxRollDegrees();

        if(driveTrain.getPose2d().getX() >= -1.2 && !arrived){
            System.out.print("---------------------------00000000000000000");
            driveTrain.setChassisSpeeds(new ChassisSpeeds(-0.45, 0, 0));
            
        }else{
          arrived= true;
          driveTrain.setChassisSpeeds(new ChassisSpeeds(degrees *  -1 * Constants.balancekP, 0.0,0.0));
        }
    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autos moda y rock&roll  command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  /** 
    if(Math.abs(degrees) < 5 && arrived){
      return true;
    }*/
    return false;
  }



}
