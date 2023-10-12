package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brazo extends SubsystemBase{

    private TalonFX motor1 = new TalonFX(61);
    private TalonFX motor2 = new TalonFX(62);
    

    public Brazo(int idTalon1, int idTalon2){
        this.motor1 = new TalonFX(idTalon1);
        this.motor2 = new TalonFX(idTalon2);

    }

    public void periodic(){
        

    }

    public void setPower(double speed){
        motor1.set(speed*0.6);
        motor2.set(speed*0.6);
    }
    
}
