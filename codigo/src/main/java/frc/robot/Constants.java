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

    //offset está en radianes
    public static final double rrOffset = 1.78;
    public static final double flOffset = 2.61;
    public static final double frOffset = 3.20;
    public static final double rlOffset = 5.98;
    
    public static final double maxDriveSignal = 0.3;
    public static final double driftMinimum = 0.1;

    //adelante positivo
    //isquierda positivo
    public static final double rrDistanceX = 0.315;
    public static final double rrDistanceY = -0.255;
    public static final double rlDistanceX = 0.285;
    public static final double rlDistanceY = 0.255;
    public static final double flDistanceX = -0.285;
    public static final double flDistanceY = 0.325;
    public static final double frDistanceX = -0.315;
    public static final double frDistanceY = -0.325;

    //                                            perímetro  in2m   reducción: 8.14:1
    public static final double driveRevs2Meters = 4 * pi / (39.37 * 8.14);
    //                                                     100ms a segundos
    public static final double driveRPS2MPS = driveRevs2Meters;

    public static final double moduleTurnkP = 0.23; 

  public static class OperatorConstants {
    
  }
}
