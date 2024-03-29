package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveTrain.DriveTrainSubsystemModified;


public class NavxResetCommandModified extends CommandBase {


    private DriveTrainSubsystemModified dTrain;

    public NavxResetCommandModified(DriveTrainSubsystemModified dTrain){
        this.dTrain = dTrain;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize(){

        dTrain.resetNavx();
        //dTrain.resetDriveEncodersPosition();
        //dTrain.resetOdometry();
        System.out.println("navX and drive encoders reset");
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

  
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
  
      return true;
    }
  
  }
