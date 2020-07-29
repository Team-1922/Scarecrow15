package frc.robot.Components;

import java.util.ArrayList;

//TODO: This probably needs to account for 2 dimensions
//TODO: Actually it definitely doesn't. There's no time to calibrate that much stuff
public class CalibrationMap
{
    private ArrayList<CalibrationPoint> m_calibrationPoints;

    public CalibrationMap(ArrayList<CalibrationPoint> calibrationPoints)
    {
        m_calibrationPoints = calibrationPoints;
    }

    public double getOffset(double distance)
    {
        //TODO: Add linear interpolation
        //TODO: Find a library for this
        return m_calibrationPoints.get(0).offset;
    }

    public double getNoGyroOffset()
    {
        return m_calibrationPoints.get(1).offset;
    }
}