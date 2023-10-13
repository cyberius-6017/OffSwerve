package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hombro;
import java.util.function.Supplier;

import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Garra;


public class BrazoCommand extends CommandBase{

    private Hombro hombro;
    private Garra garra;
    private Brazo brazo;
    private Supplier<Double> brazoSpeed, leftTrigger, rightTrigger;
    private Supplier<Boolean> aButton, bButton, xButton, yButton;

    private static double reqHombroPosition;
    private static double reqWristPosition;
    
    public BrazoCommand(Hombro hombro, Garra garra, Brazo brazo, Supplier<Double> leftY,  Supplier<Double> brazoSpeed, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger, Supplier<Boolean> aButton, Supplier<Boolean> bButton, Supplier<Boolean> xButton, Supplier<Boolean> yButton){
        this.hombro = hombro;
        this.garra = garra;
        this.brazo = brazo;
        this.brazoSpeed = leftY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;

        addRequirements(hombro);
        addRequirements(garra);
        addRequirements(brazo);

        reqHombroPosition = 0.04;
        reqWristPosition = 0.2;
    }



    @Override
    public void initialize() {
  
  
      System.out.println("Brazo command started");
    }

    @Override
    public void execute(){


        double rollerSpeed = rightTrigger.get()-leftTrigger.get();
        double armPower = brazoSpeed.get();

        garra.setRoller(rollerSpeed*0.8);

        if(aButton.get()){
          reqHombroPosition = 0.04;
          reqWristPosition = 2.0;
        }
        if(bButton.get()){
          reqHombroPosition = 0.25;
          reqWristPosition = 5.0;
        }
        if(yButton.get()){
          reqHombroPosition = 0.45;
          reqWristPosition = 12.0;
        }
        if(xButton.get()){
          reqHombroPosition = 0.4;
          reqWristPosition = 16.0;
        }

        //reqHombroPosition += hombroPoder.get() * 0.01;
        garra.setWristPosition(reqWristPosition);
        garra.setRoller(rollerSpeed);
        SmartDashboard.putNumber("Required hombro", reqHombroPosition);
        hombro.setPosition(reqHombroPosition);
        brazo.setPower(armPower);
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
