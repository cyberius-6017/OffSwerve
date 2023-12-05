package frc.robot.subsystems.driveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.lang.Math;

public class MauDriveKinematics {
    
    private Translation2d[] translations;

    public MauDriveKinematics(Translation2d module1Translation, Translation2d module2Translation, Translation2d module3Translation, Translation2d module4Translation){
        translations = new Translation2d[4];
        this.translations[0] = module1Translation;
        this.translations[1] = module2Translation;
        this.translations[2] = module3Translation;
        this.translations[3] = module4Translation;
    }

    public MauDriveKinematics(Translation2d[] translations){
        this.translations = translations;
    }
    

    public SwerveModuleState[] makeFieldOrientedStates(double xSpeed, double ySpeed, double zSpeed, double gyroRads, double maxSpeed){
        SwerveModuleState[] states = new SwerveModuleState[translations.length];
        double velocity = Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed);
        double vAngle = Math.atan2(ySpeed, xSpeed);
        double passAmount = 1;

        vAngle -= gyroRads;
        //ajuste a perspectiva del chasis  

         for(int i = 0; i < translations.length; i++){
             //por todos los módulos
                
            states[i] = makeModuleState(velocity, vAngle, zSpeed, translations[i]);
            if(states[i].speedMetersPerSecond > maxSpeed){
                //si se pasa
                
                if((maxSpeed/states[i].speedMetersPerSecond) < passAmount){
                    //si se pasa por más que el máximo anterior

                    passAmount = maxSpeed/states[i].speedMetersPerSecond;
                    //calculando la cantidad de reducción de velocidad
                }
            }

            
        }
        for(int i = 0; i< states.length; i++){
            states[i].speedMetersPerSecond *=  passAmount;
            //reduciendo todas las velocidades proporcionalmente
        }
        

        return states;
    }

    private SwerveModuleState makeModuleState(double velocity, double vAngle, double zSpeed, Translation2d translation){
        SwerveModuleState module;

        zSpeed = zSpeed * translation.getNorm();
         //ajustando proporcional a la distancia al centro

        double turnX = Math.cos(translation.getAngle().getRadians()) * zSpeed;
        double turnY = Math.sin(translation.getAngle().getRadians()) * zSpeed;
        //convertiendo velocidad tangencial a cartesiano

        double xSpeed = Math.cos(vAngle) * velocity;
        double ySpeed = Math.sin(vAngle) * velocity;
         //convertiendo velocidad lineal a cartesiano

        double x = xSpeed + turnX;
        double y = ySpeed + turnY;
        //sumando los vectores

        double magnitude = Math.sqrt(x*x+y*y);
        double direction = Math.atan2(y, x);
        //regresando a polar

        module = new SwerveModuleState(magnitude, new Rotation2d(direction));
        return module;
    }

    public Translation2d[] getTranslations(){
        return translations;
    }
}
