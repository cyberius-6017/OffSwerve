package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hombro extends SubsystemBase{

    private CANSparkMax star1;
    private CANSparkMax star2; 
    private CANSparkMax rats1; 
    private CANSparkMax rats2; 
    private SparkMaxAbsoluteEncoder encoder;
    private SparkMaxPIDController pid;

    public Hombro(int idStar1, int idStar2, int idRats1, int idRats2){
        this.star1 = new CANSparkMax(idStar1, MotorType.kBrushless);
        this.star2 = new CANSparkMax(idStar2, MotorType.kBrushless);
        this.rats1 = new CANSparkMax(idRats1, MotorType.kBrushless);
        this.rats2 = new CANSparkMax(idRats2, MotorType.kBrushless);

        star2.follow(star1, false);
        rats1.follow(star1, true);
        rats2.follow(star1, true);

        encoder = star1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        pid = star1.getPIDController();
        
        pid.setFeedbackDevice(encoder);
    }

    public double getPostion(){
        return encoder.getPosition();
    }

    public void set(double speed){
        star1.set(speed);
    }

    public void setPosition(double position){
        pid.setReference(position, ControlType.kPosition);
    }

    public void setVelocity(double velocity){
        pid.setReference(velocity, ControlType.kVelocity);
    }
    
}
