package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    private Supplier<Boolean> aButton, bButton, xButton, yButton, selButton;

    private static double reqHombroPosition;
    private static double reqWristPosition;
    private static double reqArmPosition;

    private boolean isAbove;
    
    
    public BrazoCommand(Hombro hombro, Garra garra, Brazo brazo, Supplier<Double> leftY,  Supplier<Double> rightY, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger, Supplier<Boolean> aButton, Supplier<Boolean> bButton, Supplier<Boolean> xButton, Supplier<Boolean> yButton, Supplier
    <Boolean> selButton){
        this.hombro = hombro;
        this.garra = garra;
        this.brazo = brazo;
        this.brazoSpeed = rightY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;
        this.selButton = selButton;

        addRequirements(hombro);
        addRequirements(garra);
        addRequirements(brazo);

        reqHombroPosition = 0.04;
        reqWristPosition = 0.2;
        reqArmPosition = 0.3;

        isAbove = false;
    }



    @Override
    public void initialize() {
  
  
      System.out.println("Brazo command started");

    }

    @Override
    public void execute(){


        double rollerSpeed = rightTrigger.get()-leftTrigger.get();
        //double armPower = brazoSpeed.get();

        garra.setRoller(rollerSpeed*0.8);

        if(selButton.get()){
          reqHombroPosition = 0.04;
          reqWristPosition = 2.0;
          reqArmPosition = 0.3;
          isAbove = true;
        }

        if(aButton.get()){
          reqHombroPosition = 0.04;
          reqWristPosition = 8.25;
          reqArmPosition = 0.3;
          isAbove = true;
        }
        if(bButton.get()){
          reqHombroPosition = 0.25;
          reqWristPosition = 5.0;
          reqArmPosition = 0.3;
        }
        if(yButton.get()){
          //TODO CUBOS:
          reqHombroPosition = 0.4;
          reqWristPosition = 2.0;
          reqArmPosition = 8.75;
          isAbove = false;
        }
        if(xButton.get()){
          //TODO CUBOS:
          reqHombroPosition = 0.4;
          reqWristPosition = 0.3;
        }

        //reqHombroPosition += hombroPoder.get() * 0.01;

        if(!isAbove){
          hombro.setPosition(reqHombroPosition);
          if(Math.abs(hombro.getDutyCycle()) < 0.05){
            brazo.setArmPositon(reqArmPosition);
          }
        }
        else{
          brazo.setArmPositon(reqArmPosition);
          if(Math.abs(brazo.getMotorSpeed()) < 0.05){
            hombro.setPosition(reqHombroPosition);
          }
        }


        garra.setWristPosition(reqWristPosition);
        garra.setRoller(rollerSpeed);
        SmartDashboard.putNumber("Required hombro", reqHombroPosition);
        SmartDashboard.putNumber("Arm Position", brazo.getEncoderPosition());
        
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
