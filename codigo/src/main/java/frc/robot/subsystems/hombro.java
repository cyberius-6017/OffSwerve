package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        star1.restoreFactoryDefaults();
        star2.restoreFactoryDefaults();
        rats1.restoreFactoryDefaults();
        rats2.restoreFactoryDefaults();

        star1.setSmartCurrentLimit(40);
        star2.setSmartCurrentLimit(40);
        rats1.setSmartCurrentLimit(40);
        rats2.setSmartCurrentLimit(40);

        star1.setIdleMode(IdleMode.kBrake);
        star2.setIdleMode(IdleMode.kBrake);
        rats1.setIdleMode(IdleMode.kBrake);
        rats2.setIdleMode(IdleMode.kBrake);


        star1.setInverted(true);

        star2.follow(star1, false);
        rats1.follow(star1, true);
        rats2.follow(star1, true);

        encoder = star1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder.setZeroOffset(Constants.encoderOffsetHombro);

        pid = star1.getPIDController();
        
        pid.setFeedbackDevice(encoder);
        

        pid.setP(Constants.hombrokP);
        pid.setI(Constants.hombrokI);
        pid.setD(Constants.hombrokD);
        pid.setFF(0);
        pid.setOutputRange(-0.8, 0.8);
    }

    public void periodic(){
        SmartDashboard.putNumber("hombro", getAbsolutePosition());

    }

    public double getAbsolutePosition(){
        if (encoder.getPosition() > 0.7){
            return 0;
        }
        else {
            return (encoder.getPosition()) ;
        }   
    }



    public void set(double speed){ 
        star1.set(-speed);
    }

    public void setPosition(double position){
        pid.setReference(position, ControlType.kPosition);
    }

    public void setVelocity(double velocity){
        pid.setReference(velocity, ControlType.kVelocity);
    }
    
    public void setCoast(){
        star1.setIdleMode(IdleMode.kCoast);
        star2.setIdleMode(IdleMode.kCoast);
        rats1.setIdleMode(IdleMode.kCoast);
        rats2.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake(){
        star1.setIdleMode(IdleMode.kBrake);
        star2.setIdleMode(IdleMode.kBrake);
        rats1.setIdleMode(IdleMode.kBrake);
        rats2.setIdleMode(IdleMode.kBrake);
    }

}
