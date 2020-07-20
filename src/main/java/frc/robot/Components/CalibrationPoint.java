package frc.robot.Components;

public class CalibrationPoint
{
    public CalibrationPoint(double distance, double offset)
    {
        this.offset = offset;
        this.distance = distance;
    }
    public double offset;
    public double distance;
}