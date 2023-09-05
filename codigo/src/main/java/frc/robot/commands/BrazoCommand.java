package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hombro;
import java.util.function.Supplier;


public class BrazoCommand extends CommandBase{

    private Hombro hombro;
    private Supplier<Double> hombroPoder;
    
    public BrazoCommand(Hombro hombro, Supplier<Double> leftY){
        this.hombro = hombro;
        this.hombroPoder = leftY;
    }

    @Override
    public void initialize() {
  
  
      System.out.println("Brazo command started");
    }

    @Override
    public void execute(){
        hombro.set(hombroPoder.get()*0.1);
    }



}
