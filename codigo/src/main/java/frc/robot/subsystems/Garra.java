package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

public class Garra extends SubsystemBase{
    private TalonFX wrist;
    private TalonFX roller;
    //private PositionDutyCycle pid = new PositionDutyCycle(0, false, 0, 0, true);

    public Garra(int wristId, int rollerId){
       this.wrist = new TalonFX(wristId);
       this.roller = new TalonFX(rollerId);
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
}
