package frc.robot.subsystems.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MauDriveOdometry {
    private Translation2d[] positions;
    private Translation2d[] actualPositions;
    private Pose2d pose;
    private double lastUpdate;
    private double[] initialAngles;

    public MauDriveOdometry(Translation2d[] translations){
        this.positions = translations;
        actualPositions = positions;

        this.pose = new Pose2d();
        lastUpdate = Timer.getFPGATimestamp();
        setUpAngles();
    }

    public MauDriveOdometry(Translation2d[] translations, Pose2d startPose){
        this.positions = translations;
        actualPositions = positions;

        this.pose = startPose;
        lastUpdate = Timer.getFPGATimestamp();
        setUpAngles();
    }

    public MauDriveOdometry(MauDriveKinematics mKinematics, Pose2d startPose){
        this.positions = mKinematics.getTranslations();
        actualPositions = positions;

        this.pose = startPose;
        lastUpdate = Timer.getFPGATimestamp();
        setUpAngles();
    }

    public MauDriveOdometry(MauDriveKinematics mKinematics){
        this.positions = mKinematics.getTranslations();
        actualPositions = positions;

        this.pose = new Pose2d();
        lastUpdate = Timer.getFPGATimestamp();
        setUpAngles();
    }

    public void setUpAngles(){
        initialAngles = new double[positions.length - 1];

        for(int i = 0; i < initialAngles.length; i++){
            initialAngles[i] = Math.atan2(positions[i].getY()-positions[i+1].getY(), positions[i].getX()-positions[i+1].getX());
        }
    }

    public void updateWithStates(SwerveModuleState[] states, double angle){
        if(states.length != positions.length){
            System.out.println("MauDriveOdometry.updateWithStates -- número incorrecto de states para los modulos en el constructor");
            return;
        }

        Translation2d[] shifts = new Translation2d[states.length];

        double x;
        double y;

        for(int i = 0; i < states.length; i++){
            double speed = states[i].speedMetersPerSecond;
            angle = angle + states[i].angle.getRadians();

            y = speed * Math.sin(angle);
            x = speed * Math.cos(angle);

            double time = Timer.getFPGATimestamp() - lastUpdate;
            lastUpdate = Timer.getFPGATimestamp();

            x *= time;
            y *= time;

            shifts[i] = new Translation2d(x, y);
            //actualPositions[i].plus(shifts[i]);
            actualPositions[i] = new Translation2d(actualPositions[i].getX() + x,actualPositions[i].getY() + y);
        }

        SmartDashboard.putString("shift", shifts[0].toString());
        SmartDashboard.putString("actual position", actualPositions[0].toString());

        double[] currentAngles = new double[initialAngles.length];
        double currentAngle = 0;

        for(int i = 0; i < currentAngles.length; i++){
            currentAngles[i] = Math.atan2(actualPositions[i].getY()-actualPositions[i+1].getY(), actualPositions[i].getX()-actualPositions[i+1].getX());

            currentAngles[i] = currentAngles[i]-initialAngles[i];
            currentAngle += currentAngles[i];
        }
        currentAngle /= currentAngles.length;

        SmartDashboard.putNumber("current angle", currentAngle);

    
        x = 0;
        y = 0;

        for(int i = 0; i < states.length; i++){
        
            x += actualPositions[i].getX();
            y += actualPositions[i].getY();
        }
        x /= states.length;
        y /= states.length;


        SmartDashboard.putString("estimate", actualPositions[0].toString());
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        pose.exp(new Twist2d(x, y, currentAngle));

    }

    public Pose2d getPose2d(){
        return pose;
    }
    
}
