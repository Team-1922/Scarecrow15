/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import java.lang.Math;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.AnalogInput;
import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;



import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  
  private WPI_TalonSRX m_leftMaster = new WPI_TalonSRX (Constants.cDriveLeftFront);
  private WPI_TalonSRX m_leftFollower = new WPI_TalonSRX (Constants.cDriveLeftBack);
  private WPI_TalonSRX m_rightMaster = new WPI_TalonSRX (Constants.cDriveRightFront);
  private WPI_TalonSRX m_rightFollower = new WPI_TalonSRX (Constants.cDriveRightBack);

  private AHRS m_navXMP = new AHRS(SPI.Port.kMXP);
  
  boolean m_doOld = false;
  /**
   * Creates a new DriveSystem.
   */
  public DriveSubsystem() {
    super();

   /*
    if (m_doOld) {
      oldConfigure(); 
    }
    else {
      driveTrainConfigure();
    }
    */
  
  }


  public void tankDriveConfig()
  {
    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);

    m_rightFollower.set(ControlMode.Follower, m_rightMaster.getDeviceID());
    m_rightFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.set(ControlMode.Follower, m_leftMaster.getDeviceID());
    m_leftFollower.setInverted(InvertType.FollowMaster);

    		/* Configure neutral deadband */
		m_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		m_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
  }

  public void oldConfigure() {
    
    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftFollower.configFactoryDefault();


    //Add the left master talon's encoder as a remote sensor
    m_rightMaster.configRemoteFeedbackFilter(Constants.cDriveLeftFront, RemoteSensorSource.TalonSRX_SelectedSensor, 0, 10);

		/* Setup Sum signal to be used for Distance */
		m_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, 10);				// Feedback Device of Remote Talon
		m_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, 10);	// Quadrature Encoder of current Talon

    // The remote sensor is one sensor, our local quadrature is the other
    // difference is 0 - 1
    m_rightMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, 10);
    m_rightMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, 10);

    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, 10);
    //m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0,10);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 1, 10);

    // This scales the output of the PID in some way.. I don't understand how
    m_rightMaster.configSelectedFeedbackCoefficient(1, 1, 10);
    m_rightMaster.configSelectedFeedbackCoefficient(0.5, 0, 10);
    
   // If we invert the phase on this it gets the virtual sum sensor to work
    m_leftMaster.setSensorPhase(true);   
    m_rightMaster.setSensorPhase(false);
    
    //m_rightMaster.setInverted(true);

    /* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
     * We want to set this to true beacuse PID1 is coming in as negative
     * PID0 + (- PID1) = PID0 - PID1 = What we want for the aux
     * PID0 - (- PID1) = PID0 + PID1 = What we want for the local
		 */
    m_rightMaster.configAuxPIDPolarity(false, 10);

    m_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 10);
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 10);
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 10);
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, 10);
    m_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 10);
    

    /**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		m_rightMaster.configClosedLoopPeriod(0, closedLoopTimeMs, 10);
		m_rightMaster.configClosedLoopPeriod(1, closedLoopTimeMs, 10);


       // m_rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
       m_rightMaster.setSelectedSensorPosition(0, 0, 10);
       m_rightMaster.setNeutralMode(NeutralMode.Brake);
       m_rightMaster.config_kF(0, 0.3);
       m_rightMaster.config_kP(0, 0.5);
       m_rightMaster.selectProfileSlot(0, 0);
       m_rightMaster.configMotionCruiseVelocity((int)((10.0 / 600.0) * 4096.0)); // 100 rev/min * 1min/600 (100ms) * 4096 tick/rev = ticks/100ms
       m_rightMaster.configMotionAcceleration(10);
   
       m_leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
       m_leftMaster.setSelectedSensorPosition(0, 0, 10);
       m_leftMaster.setNeutralMode(NeutralMode.Brake);
       m_leftMaster.config_kF(0, 0.3);
       m_leftMaster.config_kP(0, 0.5);
       m_leftMaster.selectProfileSlot(0, 0);
       m_leftMaster.configMotionCruiseVelocity(10 / 600 * 4096); // 100 rev/min * 1min/600 (100ms) * 4096 tick/rev = ticks/100ms
       m_leftMaster.configMotionAcceleration(10);

       
    m_rightFollower.set(ControlMode.Follower, m_rightMaster.getDeviceID());
    m_rightFollower.setInverted(InvertType.FollowMaster);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.set(ControlMode.Follower, m_leftMaster.getDeviceID());
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
  
  }

  public void driveTrainConfigure()
  {



    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    m_leftMaster.set(ControlMode.PercentOutput, 0);
    m_rightMaster.set(ControlMode.PercentOutput, 0);
    m_leftFollower.set(ControlMode.PercentOutput, 0);
    m_rightFollower.set(ControlMode.PercentOutput, 0);

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);

    m_rightFollower.set(ControlMode.Follower, m_rightMaster.getDeviceID());
    m_rightFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.set(ControlMode.Follower, m_leftMaster.getDeviceID());
    m_leftFollower.setInverted(InvertType.FollowMaster);


   		/* Configure the left Talon's selected sensor as local QuadEncoder */
		m_leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
                                              Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
                                              Constants.kTimeoutMs);					// Configuration Timeout


/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
    m_rightMaster.configRemoteFeedbackFilter(m_leftMaster.getDeviceID(),					// Device ID of Source
                                            RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
                                            Constants.REMOTE_0,							// Source number [0, 1]
                                            Constants.kTimeoutMs);						// Configuration Timeout


                                            
		/* Setup Sum signal to be used for Distance */
		m_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
		m_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		m_rightMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
		m_rightMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		m_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		m_rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		m_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													Constants.PID_TURN, 
													Constants.kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		m_rightMaster.configSelectedFeedbackCoefficient(	1,
														Constants.PID_TURN, 
														Constants.kTimeoutMs);
		/* Configure output and sensor direction */
		m_leftMaster.setInverted(false);
		m_leftMaster.setSensorPhase(true);
		m_rightMaster.setInverted(true);
		m_rightMaster.setSensorPhase(false);
		
		/* Set status frame periods to ensure we don't have stale data */
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 5, Constants.kTimeoutMs);
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 5, Constants.kTimeoutMs);
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 5, Constants.kTimeoutMs);
		m_rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5, Constants.kTimeoutMs);
		m_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		m_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		m_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		
		/* Motion Magic Configurations */
		m_rightMaster.configMotionAcceleration(500, Constants.kTimeoutMs);
	//	m_rightMaster.configMotionCruiseVelocity((int)((10.0 / 600.0) * 4096.0), Constants.kTimeoutMs);
		m_rightMaster.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		m_leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		m_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    m_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
    


    		/* FPID Gains for distance servo */
		m_rightMaster.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		m_rightMaster.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		m_rightMaster.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		m_rightMaster.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		m_rightMaster.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		m_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);
		m_rightMaster.configAllowableClosedloopError(Constants.kSlot_Distanc, 0, Constants.kTimeoutMs);

		/* FPID Gains for turn servo */
		m_rightMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		m_rightMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		m_rightMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		m_rightMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		m_rightMaster.config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		m_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
		m_rightMaster.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);


		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		m_rightMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
		m_rightMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
    m_rightMaster.configAuxPIDPolarity(true, Constants.kTimeoutMs);
    
      zeroSensors();

  }

  void zeroSensors() {
		m_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		m_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders] All Drive Subsystem Encoders are zeroed.\n");
	}

  public void resetGyro()
  {
    m_navXMP.zeroYaw();
    m_navXMP.resetDisplacement();
    m_navXMP.reset();
    m_rightMaster.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftMaster.getSensorCollection().setQuadraturePosition(0, 10);
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }

  public void stop()
  {
    m_leftMaster.stopMotor();
    m_rightMaster.stopMotor();
  }

  public void drive (double left, double right)
  {
   
      m_leftMaster.set(left);
      m_rightMaster.set(right);
    
  }

  public int getLeftError()
  {
    return m_leftMaster.getClosedLoopError();
  }

  public int getRightError()
  {
    return m_rightMaster.getClosedLoopError();
  }

  public void driveToTarget(int rightTarget, int targetEncoderDistance)
  {

    //drive (.2, .2);
   // return; 

    
   if (m_doOld){
      m_leftMaster.selectProfileSlot(0, 0);
      m_rightMaster.selectProfileSlot(0, 0);
  
      m_rightMaster.follow(m_leftMaster, FollowerType.AuxOutput1);
      m_leftMaster.set(ControlMode.MotionMagic, rightTarget, DemandType.AuxPID, targetEncoderDistance);
    
    }
    else {
      //m_leftMaster.selectProfileSlot(0, 0);
     // m_rightMaster.selectProfileSlot(0, 0);

      m_rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
      m_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
  
      m_rightMaster.set(ControlMode.MotionMagic, rightTarget, DemandType.AuxPID, 0);
      m_leftMaster.follow(m_rightMaster, FollowerType.AuxOutput1);
      

     // m_rightMaster.set(ControlMode.MotionMagic, rightTarget, DemandType.AuxPID, targetEncoderDistance);
     // m_leftMaster.follow(m_rightMaster, FollowerType.AuxOutput1);
     }


    System.out.println("[drivesubsystem] - drive to target " + rightTarget + " " + targetEncoderDistance);

//    m_rightMaster.set(ControlMode.MotionMagic, rightTarget, DemandType.AuxPID, targetEncoderDistance);
//    m_leftMaster.follow(m_rightMaster, FollowerType.AuxOutput1);
  }
  
  public int getLeftEncoder() {
    return m_leftMaster.getSelectedSensorPosition();
  }
  public int getRightEncoder() {
    return m_rightMaster.getSelectedSensorPosition();
  }
  public int getEncoderSum() {
    return m_rightMaster.getSelectedSensorPosition(0);
  }

  public int getEncoderDifference() {
    return m_rightMaster.getSelectedSensorPosition(1);
  }

  public double getAngle() {
    return m_navXMP.getYaw();
  }


  
  @Override
  public void periodic() {

    SmartDashboard.putNumber ("left encoder", getLeftEncoder());
    SmartDashboard.putNumber ("right encoder", m_rightMaster.getSensorCollection().getQuadraturePosition());

    SmartDashboard.putNumber("right sensor 0", m_rightMaster.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("right sensor 1", m_rightMaster.getSelectedSensorPosition(1));

    SmartDashboard.putNumber("left sensor 0", m_leftMaster.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("left sensor 1", m_leftMaster.getSelectedSensorPosition(1));


    SmartDashboard.putNumber("right Error", m_rightMaster.getClosedLoopError());
    SmartDashboard.putNumber("left Error", m_leftMaster.getClosedLoopError());
    }
}
