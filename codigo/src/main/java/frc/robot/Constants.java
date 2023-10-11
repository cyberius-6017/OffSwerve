// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int driveControllerID = 0;
  public static final int mechanismControllerId = 1;
  public static final double pi = Math.PI;

    
    public static final int frEncoderId = 20; 
    public static final int flEncoderId = 30; 
    public static final int rrEncoderId = 40; 
    public static final int rlEncoderId = 50; 

    public static final int frDriveId = 21;
    public static final int flDriveId = 31;
    public static final int rrDriveId = 41;
    public static final int rlDriveId = 51;

    public static final int frTurnId = 22; 
    public static final int flTurnId = 32;
    public static final int rrTurnId = 42;
    public static final int rlTurnId = 52; 

    public static final double EncElbOffset = -0.7375; 
    

    //offset está en radianes
    public static final double rrOffset = 1.67 + pi;//1.54;
    public static final double flOffset = 1.3 ;//2.61 - pi/2 ;
    public static final double frOffset = 0.29;//3.20 + pi;
    public static final double rlOffset = 1.05+ pi;//1.23;
    
    public static final double maxDriveSignal = 0.85;
    public static final double driftMinimum = 0.1;

    /////////////MECANISMOS////////////////////////
    //TODO: Revisar ID's

    // 1 y 2 van juntos, 3 y 4 van juntos
    public static final int hombroId1 = 10;
    public static final int hombroId2 = 11;
    public static final int hombroId3 = 12;
    public static final int hombroId4 = 13;

    public static final int wristId = 60;
    public static final int rollerId = 17;

    //adelante positivo
    //izquierda positivo
    public static final double rrDistanceX = 0.593;
    public static final double rrDistanceY = -0.593;
    public static final double rlDistanceX = 0.593;
    public static final double rlDistanceY = 0.593;
    public static final double flDistanceX = -0.593;
    public static final double flDistanceY = 0.593; 
    public static final double frDistanceX = -0.593;
    public static final double frDistanceY = -0.593;

    //                                            perímetro  in2m   reducción: 8.14:1
    public static final double driveRevs2Meters = 4 * pi / (39.37 * 8.14);
    //                                                     100ms a segundos
    public static final double driveRPS2MPS = driveRevs2Meters;
    public static final double falconMaxFrequency = 100; // por cambiar

    public static final double balancekP = 1/180 * 5;

    public static final double elevadorRevs2Meters = (11/50) * (18*0.005) * 2; //checar valores
                                    //polea chica a grande * polea a movimiento de banda * 2 etapas

    public static final double wristRevs2Rev = (9/50) * (15/40) * (15/40);



    public static final double moduleTurnkP = 0.23; 


    public static final double encoderOffsetHombro =  0.9;
    public static final double hombrokP = 1.2;
    public static final double hombrokI = 0.0;
    public static final double hombrokD = 0.00002;

  public static class OperatorConstants {
    
  }
}
