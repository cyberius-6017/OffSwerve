package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;
import frc.robot.subsystems.Hombro;

import java.io.Serial;
import java.util.function.Supplier;


public class AutoCommand extends CommandBase{

    private DriveTrainSubsystemModified driveTrain;

    
    
    public AutoCommand(DriveTrainSubsystemModified driveTrain){
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
  
  
      System.out.println("Autos moda y rock and roll command started");
    }

    @Override
    public void execute(){
        if(driveTrain.getPose2d().getX() >= -1){
            System.out.print("---------------------------0000000000000");
            driveTrain.setChassisSpeeds(new ChassisSpeeds(-0.2, 0, 0));
        }else{
          driveTrain.stopModules();
        }
    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autos moda y rock command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }



}
