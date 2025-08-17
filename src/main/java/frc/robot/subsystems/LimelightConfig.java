package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Components;

public class LimelightConfig {
    
    NetworkTable limelight;

    public static LimelightConfig mInstance = new LimelightConfig(Components.LIMELIGHT);

    public LimelightConfig(String table){
        limelight = NetworkTableInstance.getDefault().getTable(table);
    }

    public static LimelightConfig getInstance(){
        if(mInstance ==  null){
            return new LimelightConfig(Components.LIMELIGHT);
        }
        return mInstance;
    }

    public boolean getHasTarget(){
        return limelight.getEntry("tv").getDouble(0)==1;
    }

    public double getTagId(){
        return limelight.getEntry("tid").getDouble(-1);
    }

    public double getTx(){
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getTy(){
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getTa(){
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public boolean setLedMode(int mode){
        return limelight.getEntry("ledMode").setNumber(mode);
    }

    public double[] getRobotPose(){
        return limelight.getEntry("botPose").getDoubleArray(new double[6]);
    }

    public double[] getAprilTagCordenates(){
        return limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }
}
