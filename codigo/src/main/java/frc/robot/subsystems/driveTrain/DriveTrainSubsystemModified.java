// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveTrain;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;




public class DriveTrainSubsystemModified extends SubsystemBase {

  private SwerveModule flModule = new SwerveModule(Constants.flTurnId, Constants.flDriveId, 
  Constants.flEncoderId, Constants.flDistanceX, Constants.flDistanceY, Constants.flOffset, false);
  
 
  private SwerveModule frModule = new SwerveModule(Constants.frTurnId, Constants.frDriveId, 
  Constants.frEncoderId, Constants.frDistanceX, Constants.frDistanceY, Constants.frOffset, false);
 
  private SwerveModule rlModule = new SwerveModule(Constants.rlTurnId, Constants.rlDriveId, 
  Constants.rlEncoderId, Constants.rlDistanceX, Constants.rlDistanceY, Constants.rlOffset, false);

  private SwerveModule rrModule = new SwerveModule(Constants.rrTurnId, Constants.rrDriveId, 
  Constants.rrEncoderId, Constants.rrDistanceX, Constants.rrDistanceY, Constants.rrOffset, false);

  private AHRS navx = new AHRS();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(flModule.getTranslation2d(), 
  frModule.getTranslation2d(), rlModule.getTranslation2d(), rrModule.getTranslation2d());

  private MauDriveKinematics mKinematics = new MauDriveKinematics(Constants.flTranslationMau, 
  Constants.frTranslationMau, Constants.rlTranslationMau, Constants.rrTranslationMau);
  
  private SwerveModulePosition[] positions = {flModule.getSwervePosition(), frModule.getSwervePosition(), 
                   rlModule.getSwervePosition(), rrModule.getSwervePosition()};

  private SwerveModuleState[] states = {flModule.getSwerveState(), frModule.getSwerveState(), 
                    rlModule.getSwerveState(), rrModule.getSwerveState()};

  private SwerveDriveOdometry odometry;
  private MauDriveOdometry mOdometry;

  private SwerveDrivePoseEstimator poseEstimator;

  private Supplier<Boolean> brakeButton;
  private double[] poseNums = new double[6];
  private Pose2d visionMeasurement;
  private Field2d field2d = new Field2d();

  public DriveTrainSubsystemModified() {

    //reset yaw navx sin interrumpir código
    //es posible que interfiera en odometría
     
      navx.reset();
      
      odometry = new SwerveDriveOdometry(kinematics, getPoseRotation2d(), positions);
      mOdometry = new MauDriveOdometry(mKinematics);

      poseEstimator = new SwerveDrivePoseEstimator(kinematics, getPoseRotation2d(), positions, new Pose2d(0, 0, getPoseRotation2d()));

   
  }

  public void resetNavx(){
    navx.reset();
  }

  public void calibrateNavx(){
    navx.calibrate();
  }

  public double getNavxYawRadians(){
    double angle = navx.getYaw();
    angle += 180;
    //angle *= -1;
    angle *= Constants.pi / 180;
    angle -=  Constants.pi/2;
    if (angle > 2 * Constants.pi){
      angle -= 2* Constants.pi;
    }

    if (angle < 0){
      angle += 2 * Constants.pi;
    }
    
    return angle;
  }

  public double getNavxPitchDegrees(){
    return navx.getPitch();
  }

  public double getNavxRollDegrees(){
    return navx.getRoll();
  }

  public Rotation2d getRotation2d(){
    return new Rotation2d(getNavxYawRadians());
  }

  public void stopModules(){
    flModule.stop();
    frModule.stop();
    rlModule.stop();
    rrModule.stop();
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxDriveSignal);
    states[0].speedMetersPerSecond = -states[0].speedMetersPerSecond; //voltear drive motor de fl porque así es

   
    flModule.setSwerveState(states[0]);
    frModule.setSwerveState(states[1]);
    rlModule.setSwerveState(states[2]);
    rrModule.setSwerveState(states[3]);

    SmartDashboard.putNumber("fl req", states[0].angle.getRadians());
    SmartDashboard.putNumber("fr req", states[1].angle.getRadians());
    SmartDashboard.putNumber("rl req", states[2].angle.getRadians());
    SmartDashboard.putNumber("rr req", states[3].angle.getRadians());
  }

  public void setChassisSpeeds(ChassisSpeeds speeds){
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public void setLimitedChassisSpeeds(ChassisSpeeds speeds, double maxSpeed){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed); 

   
    flModule.setSwerveState(states[0]);
    frModule.setSwerveState(states[1]);
    rlModule.setSwerveState(states[2]);
    rrModule.setSwerveState(states[3]);
  }

  public void setFieldOrientedSpeeds(double xSpeed, double ySpeed, double zSpeed){
    double minimum = Constants.driftMinimum;

    if(xSpeed > 0){
      xSpeed -= minimum;
    } else {
      xSpeed += minimum;
    }
    if(xSpeed >= - minimum && xSpeed <= minimum){
      xSpeed = 0;
    } else{
      xSpeed *= (1+ minimum);
    }

    if(ySpeed > 0){
      ySpeed -= minimum;
    } else {
      ySpeed += minimum;
    }
    if(ySpeed >= -minimum && ySpeed <= minimum){
      ySpeed = 0;
    } else{
      ySpeed *= (1+ minimum);
    }

    if(zSpeed > 0){
      zSpeed -= minimum;
    } else {
      zSpeed += minimum;
    }
    if(zSpeed >= - minimum && zSpeed <= minimum){
      zSpeed = 0;
    } else{
      zSpeed *= (1+ minimum);
    }


    //setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d()));
    setMauOrientedSpeeds(xSpeed, ySpeed, zSpeed);
  }

  private void setMauOrientedSpeeds(double xSpeed,double ySpeed,double zSpeed){

    SwerveModuleState[] states = mKinematics.makeFieldOrientedStates(xSpeed, ySpeed, zSpeed, getRotation2d().getRadians(), Constants.maxDriveSignal);
    setModuleStates(states);
  }

  public void setDesaturatedFieldOrientedAutoSpeeds(double xSpeed, double ySpeed, double zSpeed, double maxSpeed){
    double minimum = Constants.driftMinimum * 0.5;

    if(xSpeed >= - minimum && xSpeed <= minimum){
      xSpeed = 0;
    }
    if(ySpeed >= -minimum && ySpeed <= minimum){
      ySpeed = 0;
    }
    if(zSpeed >= - minimum && zSpeed <= minimum){
      zSpeed = 0;
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(states ,maxSpeed);
    setModuleStates(states);
  }

  public void setNormalSpeeds(double xSpeed, double ySpeed, double zSpeed, double maxSpeed){
    double minimum = Constants.driftMinimum;

    if(xSpeed > 0){
      xSpeed -= minimum;
    } else {
      xSpeed += minimum;
    }
    if(xSpeed >= - minimum && xSpeed <= minimum){
      xSpeed = 0;
    } else{
      xSpeed *= (1+ minimum);
    }

    if(ySpeed > 0){
      ySpeed -= minimum;
    } else {
      ySpeed += minimum;
    }
    if(ySpeed >= -minimum && ySpeed <= minimum){
      ySpeed = 0;
    } else{
      ySpeed *= (1+ minimum);
    }

    if(zSpeed > 0){
      zSpeed -= minimum;
    } else {
      zSpeed += minimum;
    }
    if(zSpeed >= - minimum && zSpeed <= minimum){
      zSpeed = 0;
    } else{
      zSpeed *= (1+ minimum);
    }

    setChassisSpeeds(new ChassisSpeeds(-xSpeed, -ySpeed, zSpeed));
    setLimitedChassisSpeeds(new ChassisSpeeds(-xSpeed, -ySpeed, zSpeed), maxSpeed);
  }


  public void periodic(){
    
    positions[0] = flModule.getSwervePosition();
    positions[1] = frModule.getSwervePosition(); 
    positions[2] = rlModule.getSwervePosition();
    positions[3] = rrModule.getSwervePosition(); 
    odometry.update(getRotation2d(), positions);
    poseEstimator.update(getRotation2d(), positions);

    states[0] = flModule.getSwerveState();
    states[1] = frModule.getSwerveState();
    states[2] = rlModule.getSwerveState();
    states[3] = rrModule.getSwerveState();
    mOdometry.updateWithStates(states, getRotation2d().getRadians());


    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    
    if(table.getEntry("tv").getDouble(0) == 1){
      poseNums = table.getEntry("botpose_wpiblue").getDoubleArray(poseNums);

      visionMeasurement = new Pose2d(poseNums[0], poseNums[1], getPoseRotation2d());
      poseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());
    }

    toggleBrake();

    field2d.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData(field2d);
    SmartDashboard.putString("Pose Estimate", poseEstimator.getEstimatedPosition().getTranslation().toString());
    SmartDashboard.putString("Odometry  Pose", getPose2d().getTranslation().toString());
    SmartDashboard.putString("mau Pose", mOdometry.getPose2d().toString());
  }

  public void setOdoToLimelight(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    poseNums = table.getEntry("botpose_wpiblue").getDoubleArray(poseNums);

    visionMeasurement = new Pose2d(poseNums[0], poseNums[1], getPoseRotation2d());
    double rotation = getPoseRotation2d().getRadians();
    rotation += Constants.pi/2;
    if(rotation > 2*Constants.pi){
      rotation-= 2*Constants.pi;
    }
    odometry.resetPosition(new Rotation2d(rotation), positions, visionMeasurement);
  }

  public void updateOdo(){
    positions[0] = flModule.getSwervePosition();
    positions[1] = frModule.getSwervePosition(); 
    positions[2] = rlModule.getSwervePosition();
    positions[3] = rrModule.getSwervePosition(); 
    odometry.update(getPoseRotation2d(), positions);
    poseEstimator.update(getPoseRotation2d(), positions);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    if(table.getEntry("tv").getDouble(0) == 1){
      poseNums = table.getEntry("botpose_wpiblue").getDoubleArray(poseNums);

      visionMeasurement = new Pose2d(poseNums[0], poseNums[1], getRotation2d());
      poseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp());
    }
    field2d.setRobotPose(poseEstimator.getEstimatedPosition());

  }

  public Rotation2d getPoseRotation2d(){
    return getRotation2d();
  }

  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(){
     odometry.resetPosition(getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));
     poseEstimator.resetPosition(getRotation2d(), positions, new Pose2d(0,0, getRotation2d()));
  }


  public void setBrake(){
    flModule.setBrake();
    frModule.setBrake();
    rlModule.setBrake();
    rrModule.setBrake();
  }

  public void setCoast(){
    flModule.setCoast();
    frModule.setCoast();
    rlModule.setCoast();
    rrModule.setCoast();
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
