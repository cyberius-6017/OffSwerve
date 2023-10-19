package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Supplier;

public class Garra extends SubsystemBase{
    private TalonFX wrist;
    private TalonFX roller;
    private Supplier<Boolean> brakeButton;
    

    public Garra(int wristId, int rollerId, Supplier<Boolean> brakeButton){
       this.wrist = new TalonFX(wristId);
       this.roller = new TalonFX(rollerId);

       this.brakeButton = brakeButton;
    }

    public void setWrist(double speed){
       wrist.set(speed);

       if(Math.abs(speed) <0.05){
        wrist.setControl(new StaticBrake());
       }
    }

    public void setRoller(double speed){
       roller.set(speed);
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
