/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
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

	// Ultrasonic
	public static final int kLeftUltrasonicPort = 0;
	public static final int kRightUltrasonicPort = 1;

	public static final double kValueToInches = 0.125; // 1/8" inch 8 units in an inch
	public static final double kMMinInch = 25.4; // 25 mm in an inch-

	// TOF
	public static final int kLeftTOF = 0; //
	public static final int kRightTOF = 1; //
	public static final double kTOFTimingSampling = 200; // it will sample every 200 milliseconds

	// canifier (LED lights)
	public static final int kCanifier0 = 4; // first canifier
	public static final int kLEDOzRamRed = 255;
	public static final int kLEDOzRamGreen = 191;
	public static final int kLEDOzRamBlue = 0;

	// IMU
	public static final int kIMU = 7;


	// Beam break
	public static final int kBeamBreak = 0;

	///////////// from CTRE

	/**
	 * How many sensor units per rotation. Using CTRE Magnetic Encoder.
	 * 
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	public final static int kSensorUnitsPerRotation = 4096;

	/**
	 * Number of rotations to drive when performing Distance Closed Loop
	 */
	public final static double kRotationsToTravel = 6;

	/**
	 * Empirically measure what the difference between encoders per 360' Drive the
	 * robot in clockwise rotations and measure the units per rotation. Drive the
	 * robot in counter clockwise rotations and measure the units per rotation. Take
	 * the average of the two.
	 */
	// public final static int kEncoderUnitsPerRotation = 51711;
	/**
	 * Set to zero to skip waiting for confirmation. Set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public final static int kTimeoutMs = 10;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.0401;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control
	 * loop. kF: 1023 represents output value to Talon at 100%, 6800 represents
	 * Velocity units at 100% output Not all set of Gains are used in this project
	 * and may be removed as desired.
	 * 
	 * kP kI kD kF Iz PeakOut
	 */

	 
    // public final static Gains kGains_Velocit = new Gains( 0.25, 0.001, 20,
	// 1023.0/7200.0, 300, 1.00);
	
	public final static Gains kGains_Distanc = new Gains(0.3, 1023.0 / 7200.0, 1, 0.177, 100, 0.50);
	// public final static Gains kGains_Distanc = new Gains(0.5, 0.000, 1, 0.2, 100, 0.50);
	public final static Gains kGains_Turning = new Gains(0.0, 1023.0 / 7200.0, 0.0, 0.177, 200, 1.00);

	// public final static Gains kGains_Velocit = new Gains( 0.25, 0.001, 20,
	// 1023.0/7200.0, 300, 1.00);
	public final static Gains kGains_Velocit = new Gains(0.0, 0.00, 0, 0.186, 300, 1.00);

	/** ---- Flat constants, you should not need to change these ---- */
	/*
	 * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
	 * can have up to 2 devices assigned remotely to a talon/victor]
	 */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/*
	 * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
	 * is auxiliary
	 */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/*
	 * Firmware currently supports slots [0, 3] and can be used for either PID Set
	 */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;


	public final static double kWheelDiameterInches = 3.943; //
	public final static double kFullRotationPulses = 4096.0;
	public final static double kWheelDiameter = Units.inchesToMeters(kWheelDiameterInches); // wheel diameter in meters
	public final static double kWheelCircumference = kWheelDiameter * Math.PI; // circumference in meters
	public final static double kEncoderTicksPerMeter = kFullRotationPulses * (1.0 / kWheelCircumference);
	public final static double kMetersPerTic = (1.0 /  kWheelCircumference) / kFullRotationPulses;


	public final static double kAutonomousMaxSpeedMetersPerSecond = 0.5; // this can go up to 3 meters/second to give us 10ft/second
	public final static double kAutoMaxAccelerationMetersPerSecondSquared = 0.25; // wild guess at a reasonable value, needs to be tuned
	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
	public final static double kRamseteB = 2;
	public final static double kRamseteZeta = 0.75;


	public static final double kTrackwidthMeters = (22.0 * .0254); 
	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
	
	public static final double kMaxTelopVelocity = Units.feetToMeters(9.0); // 12 feet per second - needs to match the current gearbox capability
	public static final double kMaxTelopAcceleration = Units.feetToMeters(3.0); // 12 meters per second
	public static final int kMaxTelopAccelerationInSensorUnits = (int) ((kMaxTelopAcceleration / kEncoderTicksPerMeter) * 0.1);  // the ten is to get it to 100ms instead of a second


}
