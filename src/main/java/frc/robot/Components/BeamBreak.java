package frc.robot.Components;

import java.lang.Math;

import edu.wpi.first.wpilibj.AnalogInput;

public class BeamBreak extends AnalogInput {

  
    private int m_broken = -1; // -1 indicates that it is indeterminant
    private int m_floor = 5000; // sentinel - all the values should be less than this
    private int m_ceiling = -5000;
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

        if (accumulatorCount > 10000 ){  // have a good number of samples, let's determine the range

            if (averageValue > m_ceiling || averageValue < m_floor) {
                m_broken = -1;
                if (averageValue > m_ceiling ) { 
                    if (m_ceiling == m_floor) {
                        m_broken = 1;
                    }
                        
                    m_ceiling = averageValue; 
                    }
                if (averageValue < m_floor ) { m_floor = averageValue; }
            }
            else {
                int range = m_ceiling - m_floor;
                int position = averageValue + 1 - m_floor;
                if (Math.abs(position) < range) {
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

        
        //SmartDashboard.putNumber("beam break", getAverageValue());
        //SmartDashboard.putNumber("floor", m_floor);
        //SmartDashboard.putNumber("ceiling", m_ceiling);
        
        //SmartDashboard.putBoolean("Beam Broken", broken());
        
    }

    public boolean broken() {
        return m_broken > 0;
    }


}
