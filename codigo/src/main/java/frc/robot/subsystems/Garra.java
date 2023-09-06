package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Garra extends SubsystemBase{
    private TalonFX wrist;
    private TalonFX roller;

    public Garra(int wristId, int rollerId){
        this.wrist = new TalonFX(wristId);
        this.roller = new TalonFX(rollerId);
    }

    public void setWrist(double speed){
        wrist.set(speed);
    }

    public void setRoller(double speed){
        roller.set(speed);
    }
}
