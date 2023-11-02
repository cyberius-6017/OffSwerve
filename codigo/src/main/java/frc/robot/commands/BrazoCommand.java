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
    private Supplier<Double> rightY, leftTrigger, rightTrigger;
    private Supplier<Boolean> aButton, bButton, xButton, yButton, selButton, rBump, rStickPress, leftBumper, lStickPress;

    private static double reqHombroPosition;
    private static double reqWristPosition;
    private static double reqArmPosition;

    private boolean cube;
    private boolean wasArriba;
    
    
    public BrazoCommand(Hombro hombro, Garra garra, Brazo brazo, Supplier<Double> leftY,  Supplier<Double> rightY, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger, Supplier<Boolean> aButton, Supplier<Boolean> bButton, Supplier<Boolean> xButton, Supplier<Boolean> yButton, Supplier
    <Boolean> selButton, Supplier<Boolean>rBump, Supplier<Boolean> rStickPress, Supplier<Boolean> leftBumper, Supplier<Boolean> lStickPress){
        this.hombro = hombro;
        this.garra = garra;
        this.brazo = brazo;
        this.rightY = rightY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;
        this.selButton = selButton;
        this.rBump = rBump;
        this.rStickPress = rStickPress;
        this.leftBumper = leftBumper;
        this.lStickPress = lStickPress;
      

        addRequirements(hombro);
        addRequirements(garra);
        addRequirements(brazo);


        reqHombroPosition = 0.04;
        reqWristPosition = 0.2;
        reqArmPosition = 0.3;
        // TRUE = 
        cube = true;
        wasArriba = false;
    }



    @Override
    public void initialize() {
  
  
      System.out.println("Brazo command started");

    }

    @Override
    public void execute(){
        //CONE CUBE
        SmartDashboard.putBoolean("CUBE/CONE", cube);
        if(rBump.get()){

          cube = !cube;
        }

        double rollerSpeed = rightTrigger.get()-leftTrigger.get();
        //double armPower = brazoSpeed.get();


        if(selButton.get()){
          reqHombroPosition = 0.004;
          reqWristPosition = 2.0;
          reqArmPosition = 0.4;
          brazo.setArmPositon(reqArmPosition);
          
          if(wasArriba){
            Timer.delay(0.3);
            hombro.setPosition(reqHombroPosition);
            Timer.delay(0.5);
            brazo.setReading(1);
          } else{
            hombro.setPosition(reqHombroPosition);
          }
          wasArriba = false;
        }

        if(aButton.get()){
          if(cube){
            reqHombroPosition = 0.004;
            reqWristPosition = 9.75;
            reqArmPosition = 0.4;
          }
          else{
            reqHombroPosition = 0.04;
            reqWristPosition = 13.9;
            reqArmPosition = 0.4;
          }
          brazo.setArmPositon(reqArmPosition);
          
          if(wasArriba){
            Timer.delay(0.3);
            hombro.setPosition(reqHombroPosition);
            Timer.delay(1);
            brazo.setReading(0.4);
          } else{
            hombro.setPosition(reqHombroPosition);
          }
          wasArriba = false;
        }
        if(bButton.get()){
          
          if(cube){
            reqHombroPosition = 0.4;
            reqWristPosition = 0.1;
            reqArmPosition = 0.4;
          }
          else{
            reqWristPosition = 0.1;
            reqArmPosition = 8;
            reqHombroPosition = 0.37;
          }
          hombro.setPosition(reqHombroPosition);
          Timer.delay(0.95);
          brazo.setArmPositon(reqArmPosition);
          wasArriba = true;
        }
        if(yButton.get()){
          //TODO CUBOS:
          
          
          if(cube){
            reqWristPosition = 3.5;
            reqArmPosition = 5.5;
            reqHombroPosition = 0.4;
          }
          else{
            reqWristPosition = 8.5;
            reqArmPosition = 8.7;
            reqHombroPosition = 0.385;
          }
          hombro.setPosition(reqHombroPosition);
          Timer.delay(0.95);
          brazo.setArmPositon(reqArmPosition);
          wasArriba = true;
          
        }
        if(xButton.get()){
          //TODO CUBOS:
          

          if(cube){
            reqHombroPosition = 0.4;
            reqWristPosition = 0.3;
            reqArmPosition = 0.4;
          
          } else {
            reqWristPosition = 3;
            reqArmPosition = 4.5;
            reqHombroPosition = 0.37;
          }
          hombro.setPosition(reqHombroPosition);
          Timer.delay(0.95);
          brazo.setArmPositon(reqArmPosition);
          wasArriba = true;
        }
        if(lStickPress.get()){
          reqHombroPosition = 0.15;
          reqWristPosition = 2.0;
          reqArmPosition = 0.4;
          brazo.setArmPositon(reqArmPosition);
          
          if(wasArriba){
            Timer.delay(0.3);
            hombro.setPosition(reqHombroPosition);
            Timer.delay(0.5);
            brazo.setReading(1);
          } else{
            hombro.setPosition(reqHombroPosition);
          }
          wasArriba = false;
        }

        //reqHombroPosition += hombroPoder.get() * 0.01;

        if(cube){
          garra.setRoller(-rollerSpeed, cube);
        }
        else{
          garra.setRoller(rollerSpeed, cube);
        }

        if(Math.abs(rightY.get())>0.5){
          garra.setWrist(rightY.get()*0.05);
          if(rStickPress.get()){
            garra.setReading(0);
          }
        } else{
          garra.setWristPosition(reqWristPosition);
        }

        if(leftBumper.get()){
          reqArmPosition -= 0.4;
          brazo.setArmPositon(reqArmPosition);
          brazo.setReading(0.3);
        }

        

        
      
        SmartDashboard.putNumber("Required hombro", reqHombroPosition);
        SmartDashboard.putNumber("Arm Position", brazo.getEncoderPosition());
        SmartDashboard.putNumber("Duty Cycle Hombro", hombro.getDutyCycle());
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
