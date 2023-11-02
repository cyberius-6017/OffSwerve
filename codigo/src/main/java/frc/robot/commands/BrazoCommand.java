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
    private Supplier<Double> rightY, leftY, leftTrigger, rightTrigger;
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
        this.leftY = leftY;
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

    public static void setReqArm(double reqArm){
      reqArmPosition = reqArm;
    }

    public static void setReqHombro(double reqHombro){
      reqHombroPosition = reqHombro;
    }
    
    public static void setReqWrist(double reqWrist){
      reqWristPosition = reqWrist;
    }

    @Override
    public void execute(){
        //CONE CUBE
        SmartDashboard.putBoolean("CUBE/CONE", cube);
        if(rBump.get()){

          cube = !cube;
        }

        double rollerSpeed = rightTrigger.get()-leftTrigger.get();
       


        if(selButton.get()){
          
          if(cube){
            TimingThread thread = new TimingThread(0.004,0.4,2.0,false, wasArriba);
            thread.start();
          } else{
            TimingThread thread = new TimingThread(0.004,0.4,2.0,false, wasArriba);
            thread.start();
          }


          wasArriba = false;
        }

        if(aButton.get()){
         
          if(cube){
            TimingThread thread = new TimingThread(0.004,0.4,10.0,false, wasArriba);
            thread.start();
          } else{
            TimingThread thread = new TimingThread(0.004,0.4,13.9,false, wasArriba);
            thread.start();
          }

          wasArriba = false;
        }
        if(bButton.get()){
          
          if(cube){
            TimingThread thread = new TimingThread(0.2,6,6,true);
            thread.start();
          } else{
            TimingThread thread = new TimingThread(0.37,8,0.1,true);
            thread.start();
          }

          wasArriba = true;
        }
        if(yButton.get()){

          if(cube){
            TimingThread thread = new TimingThread(0.4,5.5,3.5,true);
            thread.start();
          } else{
            TimingThread thread = new TimingThread(0.385,8.7,8.5,true);
            thread.start();
          }

          wasArriba = true;
        }
        if(xButton.get()){
          

          if(cube){
            TimingThread thread = new TimingThread(0.4,0.4,0.3,true);
            thread.start();
          } else{
            TimingThread thread = new TimingThread(0.37,4.5,3,true);
            thread.start();
          }

          wasArriba = true;
        }
        if(lStickPress.get()){
          
          if(cube){
            TimingThread thread = new TimingThread(0.15,0.4,2.0,false, wasArriba);
            thread.start();
          } else{
            TimingThread thread = new TimingThread(0.15,0.4,2.0,false, wasArriba);
            thread.start();
          }

          wasArriba = false;
        }

        if(cube){
          garra.setRoller(-rollerSpeed, cube);
        }
        else{
          garra.setRoller(rollerSpeed, cube);
        }

        if(Math.abs(rightY.get())>0.5){
         reqWristPosition += rightY.get()*0.02;
        } 
        if(rStickPress.get()){
          garra.setReading(0);
        }

        if(Math.abs(leftY.get()) >0.5){
          reqArmPosition += leftY.get() * 0.01;
        }
        
        if(leftBumper.get()){
          brazo.setReading(0.3);
        }
       

        hombro.setPosition(reqHombroPosition);
        brazo.setArmPositon(reqArmPosition);
        garra.setWristPosition(reqWristPosition);


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




class TimingThread extends Thread{
  double hombro;
  double arm;
  double wrist;
  boolean goingUp;
  boolean wasUp = false;

  public TimingThread(double hombro, double arm, double wrist, boolean goingUp){
    this.hombro = hombro;
    this.arm = arm;
    this.wrist = wrist;
    this.goingUp = goingUp;
  }

  public TimingThread(double hombro, double arm, double wrist, boolean goingUp, boolean wasUp){
    this.hombro = hombro;
    this.arm = arm;
    this.wrist = wrist;
    this.goingUp = goingUp;
    this.wasUp = wasUp;
  }

  public void run(){
    try{
      if(goingUp){
          BrazoCommand.setReqWrist(wrist);
          BrazoCommand.setReqHombro(hombro);
          Thread.sleep(950);
          BrazoCommand.setReqArm(arm);

      } else{
        BrazoCommand.setReqArm(arm);
        BrazoCommand.setReqWrist(wrist);

        if(wasUp){
          Thread.sleep(300);
          BrazoCommand.setReqHombro(hombro);

        }else{
          BrazoCommand.setReqHombro(hombro);
        }
      }
    }catch(Exception e){
      
    }
  }
}
