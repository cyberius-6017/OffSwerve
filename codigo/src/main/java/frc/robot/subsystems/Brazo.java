package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brazo extends SubsystemBase{

    private TalonFX motor1 = new TalonFX(62);
    private TalonFX motor2 = new TalonFX(61);
    
    
    
    public Brazo(int idTalon1, int idTalon2){
        this.motor1 = new TalonFX(idTalon1);
        this.motor2 = new TalonFX(idTalon2);

        motor1.setInverted(true);
        motor2.setInverted(true);

        motor2.setControl(new StrictFollower(62));

    }

    public void periodic(){
        
        SmartDashboard.putNumber("Arm Position", getEncoderPosition());
        SmartDashboard.putNumber("Duty Cycle", getMotorSpeed());
    }

    public void setPower(double speed){
        motor1.set(speed*0.6);
        motor2.set(speed*0.6);
    }

    public void setArmPositon(double position){
        motor1.setControl(new PositionDutyCycle(position, false, 0.0, 0, true));
        motor2.setControl(new StrictFollower(62));

    } 
    public double getEncoderPosition(){
        return motor1.getRotorPosition().getValue();
    }
    public double getMotorSpeed(){
        return motor1.getDutyCycle().getValue();
    }
    
}
