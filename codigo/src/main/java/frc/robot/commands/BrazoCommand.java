package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hombro;
import java.util.function.Supplier;
import frc.robot.subsystems.Garra;


public class BrazoCommand extends CommandBase{

    private Hombro hombro;
    private Garra garra;
    private Supplier<Double> hombroPoder, wristSpeed, leftTrigger, rightTrigger;
    
    public BrazoCommand(Hombro hombro, Garra gara, Supplier<Double> leftY, Supplier<Double> wristSpeed, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger){
        this.hombro = hombro;
        this.hombroPoder = leftY;
        this.wristSpeed = wristSpeed;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;

        addRequirements(hombro);
    }

    @Override
    public void initialize() {
  
  
      System.out.println("Brazo command started");
    }

    @Override
    public void execute(){
        hombro.set(hombroPoder.get()*0.3);


        garra.setWrist(wristSpeed.get() * 0.2);

        double rollerSpeed = rightTrigger.get()-leftTrigger.get();
        
        garra.setRoller(rollerSpeed*0.6);
    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Brazo command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }



}
