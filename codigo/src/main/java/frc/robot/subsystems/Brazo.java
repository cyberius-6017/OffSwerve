package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brazo extends SubsystemBase{

    private TalonFX motor1 = new TalonFX(61);
    private TalonFX motor2 = new TalonFX(62);
    

    public Brazo(int idTalon1, int idTalon2){
        this.motor1 = new TalonFX(idTalon1);
        this.motor2 = new TalonFX(idTalon2);

        motor1.setControl(new StaticBrake());
        motor2.setControl(new StaticBrake());
    }

    public void periodic(){
        

    }

    public void setCoast(){
        motor1.setControl(new CoastOut());
        motor2.setControl(new CoastOut());
    }

    public void setBrake(){
        motor1.setControl(new StaticBrake());
        motor2.setControl(new StaticBrake());
    }

    public void setPower(double speed){
        motor1.set(speed*0.6);
        motor2.set(speed*0.6);
    }

    private boolean isBrake = true;
    private boolean wasTrue = false;
    private DigitalInput brakeButton = new DigitalInput(Constants.brakeButtonPort);
    public void toggleBrake(){

        if(!wasTrue && brakeButton.get()){

        if(isBrake){
            setCoast();
            isBrake = false;
        }else{
            setBrake();
            isBrake = true;
        }

        } 

        wasTrue = brakeButton.get();

    }
    
}
