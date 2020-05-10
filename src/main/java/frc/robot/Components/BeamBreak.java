package frc.robot.Components;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import java.lang.Math;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BeamBreak extends AnalogInput {

  
    private long m_lastCount = -1;
    private int m_broken = -1; // -1 indicates that it is indeterminant
    private int m_floor = 5000; // sentinel - all the values should be less than this
    /**
     * Creates a new DriveSystem.
     */
    public BeamBreak(int channel) {
        super (channel);
        init();
    }

    public void init() {

        resetAccumulator();
        setOversampleBits(16);
    }

    public void periodic() {

        int averageValue = getAverageValue();
        long accumulatorCount = getAccumulatorCount();

        if (accumulatorCount > 20 ){  // have a good number of samples, let's determine the range
       
            if (averageValue < m_floor) {
                m_broken =  -1;
                m_floor = averageValue;
            }
            else {

                if (averageValue > (m_floor + 1)) {
                    m_broken = 1;
                }
                else {
                    m_broken = 0;
                }
            
            }
        }
       
        report();


    }

    public void report() {
        SmartDashboard.putNumber("beam break", getAverageValue());
        SmartDashboard.putBoolean("Beam Broken", broken());
    }

    public boolean broken() {
        return m_broken > 0;
    }


}
