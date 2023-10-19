package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;


public class Brazo extends SubsystemBase{

    private TalonFX motor1 = new TalonFX(47);
    private TalonFX motor2 = new TalonFX(48);
    private Supplier<Boolean> brakeButton;
    
    
    
    public Brazo(int idTalon1, int idTalon2, Supplier<Boolean> brakeButton){
        this.motor1 = new TalonFX(idTalon1);
        this.motor2 = new TalonFX(idTalon2);

        motor1.setInverted(true);
        motor2.setInverted(true);

        motor2.setControl(new StrictFollower(47));

        this.brakeButton = brakeButton;

    }

    public void periodic(){
        
        SmartDashboard.putNumber("Arm Position", getEncoderPosition());
        SmartDashboard.putNumber("Duty Cycle", getMotorSpeed());

        toggleBrake();
    }

    public void setPower(double speed){
        motor1.set(speed*0.6);
        motor2.set(speed*0.6);
    }

    public void setArmPositon(double position){
        motor1.setControl(new PositionDutyCycle(position, false, 0.0, 0, true));
        motor2.setControl(new StrictFollower(47));

    } 

    public void setReading(double position){
        motor1.setRotorPosition(position);

    }

    public double getEncoderPosition(){
        return motor1.getRotorPosition().getValue();
    }
    public double getMotorSpeed(){
        return motor1.getDutyCycle().getValue();
    }

    public void setCoast(){
        motor1.setControl(new CoastOut());
        motor2.setControl(new CoastOut());
    }

    public void setBrake(){
        motor1.setControl(new StaticBrake());
        motor2.setControl(new StaticBrake());
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
