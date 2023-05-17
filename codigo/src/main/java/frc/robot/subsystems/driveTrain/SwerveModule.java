package frc.robot.subsystems.driveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase{
    private TalonFX driveMotor;
    private CANSparkMax turnMotor; 
    private Translation2d translation;
    private CANCoder turnEncoder;
    private double encoderOffset;
    private static boolean invEncoder;

    private static int reverse = 1;

    public SwerveModule(int turnId, int driveId, int encoderId, double xDistance, double yDistance, double encoderOffset , boolean invertEncoder){
        this.turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        this.driveMotor = new TalonFX(driveId);
        this.translation = new Translation2d(xDistance, yDistance);
        this.turnEncoder = new CANCoder(encoderId);
        //this.turnEncoder.setPositionConversionFactor(360.0 / 4096.0);
        this.encoderOffset = encoderOffset;
        invEncoder = invertEncoder;

        driveMotor.setInverted(false);
        turnMotor.setInverted(false);

        resetDriveEncoder();
    }

  
    public void setInvertedTurnMotor(boolean isInverted){
      turnMotor.setInverted(isInverted);
    }

    public void setInvertedDriveMotor(boolean isInverted){
      driveMotor.setInverted(isInverted);
    }

    public Translation2d getTranslation2d(){
        return translation;
    }

    public void resetDriveEncoder(){
        driveMotor.setSelectedSensorPosition(0);
    }

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition() * Constants.driveTicks2Meters;
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity() * Constants.driveTPD2MPS;
    }
    
    public SwerveModulePosition getSwervePosition(){
      return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAdjRadians()));
    }

    public SwerveModuleState getSwerveState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAdjRadians()));
    }


    public void setSwerveState(SwerveModuleState state){



       
        double dir = state.angle.getRadians(); // + 3 * Constants.pi / 2;

        dir = 2 * Constants.pi - dir;

        //arreglar
        if (dir > 2 * Constants.pi){
          dir = dir - 2 * Constants.pi;
        } else if (dir < 0){
          dir = dir + 2 * Constants.pi;
        }
        //voltear
       // dir = 2 * Constants.pi - dir;

        SmartDashboard.putNumber("angle" ,dir);
        SmartDashboard.putNumber("Speed" , state.speedMetersPerSecond);

        turnMotor.set(pSet(dir) * Constants.moduleTurnkP);
        if(state.speedMetersPerSecond < 0.05 && state.speedMetersPerSecond > -0.05){
          driveMotor.set(ControlMode.PercentOutput, 0);  
        } else{
          driveMotor.set(ControlMode.PercentOutput,state.speedMetersPerSecond * reverse);
        }
    }

    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(0);
    }

    public double getAdjRadians(){
        double angle = turnEncoder.getAbsolutePosition() - encoderOffset;
    
        if (angle > 360){
          angle -= 360;
        }
    
        if (angle < 0){
          angle += 360;
        }
        
        if (invEncoder){
          angle = 360 - angle;
        }

        angle = Math.toRadians(angle);
        

        return angle;
    }

    private double pSet(double reqDir){
        double rearP = 0;
        double eError = 0;
        double frontP = getAdjRadians();
        double p = 0;
      
    
        if (frontP > Constants.pi){rearP = frontP - Constants.pi;}
          else {rearP = frontP + Constants.pi;}
    
        double d1 = frontP - reqDir;
        if (d1 < 0){
          d1 = 2 * Constants.pi + d1;
          //System.out.println("1 flipped");
          //negative = -1;
        }
        double d2 = 2 * Constants.pi - d1;
        double d3 = rearP - reqDir;
        if (d3 < 0){
          d3 = 2 * Constants.pi + d3;
          //System.out.println(" 3 flipped");
    
        } 
        double d4 = 2 * Constants.pi - d3;
    
        if (d1 < d2 && d1 < d3 && d1 < d4){
          eError = d1;
          //System.out.println("1: " + d1);
          reverse =1;
        } else if (d2 < d1 && d2 < d3 && d2 < d4){
          eError = -d2;
          //System.out.println("2: " + d2);
          reverse =1;
        } else if (d3 < d1 && d3 < d2 && d3 < d4){
          eError = d3;
          reverse = -1;
          //System.out.println("3: " + d3);
        } else if (d4 < d1 && d4 < d2 && d4 < d3){
          eError = -d4;
          reverse = -1;
          //System.out.println("4: " + d4);
        }
    
    
        p = eError;
        if (p * Constants.moduleTurnkP > 1){
          p = 1 / Constants.moduleTurnkP;
          System.out.println("!!!!!!!!!!!!omggggggg!!!!!!!!!!!!!!!!!!!!!!!!! p > 1");
        }
        return p;
      }
}