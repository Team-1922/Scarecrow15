/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int cDriveLeftFront = 0;
    public static final int cDriveRightFront = 1;
    public static final int cDriveLeftBack = 2;
    public static final int cDriveRightBack = 3;

    public static final int cJoyStickLeft = 0;
    public static final int cJoyStickRight = 1;
    public static final int cXBoxController = 2;

    public static final int cXBoxYButton = 4;
    public static final int cXBoxXButton = 3;
    public static final int cXBoxAButton = 1;
    public static final int cXBoxBButton = 2;

    // 4 in wheel diameter => 12.56in circumference
    // 1rotation / 12.56 in * 4096 ticks/ 1rotation * 12in / 1ft = 3911 ticks/foot
    public static int ticksPerFoot = 3911; //

    public static double cDriveBaseDeadband = 0.02;


    // LimeLight
    public static final int cLLLedPipelineDefault = 0;
    public static final int cLLLedOff = 1;
    public static final int cLLLedBlink = 2;
    public static final int cLLLedOn = 3;
    public static final int cLLCameraVisionProcess = 0;
    public static final int cLLCameraDriver = 1;
    
    //Ultrasonic
    public static final int kLeftUltrasonicPort = 0;
    public static final int kRightUltrasonicPort = 1;

    public static final double kValueToInches = 0.125;  // 1/8" inch 8 units in an inch
    public static final double kMMinInch = 25.4; // 25 mm in an inch- 
    
    // TOF
    public static final int kLeftTOF = 0; // 
    public static final int kRightTOF = 1; // 



}
