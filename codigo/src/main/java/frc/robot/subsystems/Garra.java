package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;

public class Garra extends SubsystemBase{
    private TalonFX wrist;
    private TalonFX roller;
    private Supplier<Boolean> brakeButton;
    private double previousVelocity;
    private boolean hasPiece;

    private double startTime;

    Timer timer = new Timer();

    public Garra(int wristId, int rollerId, Supplier<Boolean> brakeButton){
       this.wrist = new TalonFX(wristId);
       this.roller = new TalonFX(rollerId);

      


       previousVelocity = 0;
       this.brakeButton = brakeButton;
       hasPiece = false;
    }

    public void setWrist(double speed){
       wrist.set(speed);

       if(Math.abs(speed) <0.05){
        wrist.setControl(new StaticBrake());
       }
    }

    public void setRoller(double speed, boolean isCube){


      double velocity = Math.abs(roller.getRotorVelocity().getValue());

      

      if(isCube){
         if(speed < 0){
            if(velocity - previousVelocity > 0 && !hasPiece){
               roller.set(speed);

            } else {
               roller.stopMotor();
               hasPiece = true;
               startTime = Timer.getFPGATimestamp();

            }
         } else{

            roller.set(speed);
            hasPiece = false;
         }
      } else{
         if(speed > 0){
            if(velocity - previousVelocity > 0 && !hasPiece){
               roller.set(speed);

            } else {
               roller.stopMotor();
               hasPiece = true;
               startTime = Timer.getFPGATimestamp();

            }
         } else{
            roller.set(speed);
            hasPiece = false;
         }
      }

      if(hasPiece){
         if(Timer.getFPGATimestamp() - startTime > 1 / Constants.gripHZ){
            if(isCube){
               roller.set(-0.1);
               timer.start();
               
            } else{
               roller.set(0.1);
               timer.start();
            }
            if(timer.get() > 0.1){
               startTime = Timer.getFPGATimestamp();
               
               timer.stop();
               timer.reset();
            }
         }
      }
       
      previousVelocity = velocity;

       
       
    }

    public void setWristPosition(double position){

      wrist.setControl(new PositionDutyCycle(position, false, 0.0, 0, true));
   }
    
   public double getWristPosition(){ 
      double position = wrist.getClosedLoopOutput().getValue();
      return position;
   }

   public void setBrake(){
      wrist.setControl(new StaticBrake());
   }

   public void setCoast(){
      wrist.setControl(new CoastOut());
   }

   public void periodic(){
      toggleBrake();
   }

   public void setReading(double position){
      wrist.setRotorPosition(position);
   }

   private boolean wasPressed = false;
   private boolean isBrake = true;

   public void toggleBrake(){

      if(!wasPressed & !brakeButton.get()){
          if(isBrake){
              setCoast();

          } else{
              setBrake();
          }

          isBrake = !isBrake;
      }
      wasPressed = !brakeButton.get();
  }



}
