package frc.robot.Components;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Planner implements Runnable {

    public void run () {
        while (true) {

            // get some values out of the network table and rewrite them to show that we can do a read and a write


            NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");
            NetworkTableEntry left = table.getEntry("TOFRight");
            double TOFRight = left.getDouble(0);

            NetworkTableEntry trackerLeft = table.getEntry("TrackerRight");
            trackerLeft.setDouble (TOFRight);

            


        }
    }

}