package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class brazo extends SubsystemBase{

    private TalonFX power1;
    private TalonFX power2; 

    public brazo(int idPower1, int idPower2){
        this.power1 = new TalonFX(Constants.armPower1Id);
        this.power2 = new TalonFX(Constants.armPower2Id);
            
    }
    
}
