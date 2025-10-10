package frc.FRC9485.utils.logger.logEntry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class Pose2dLogEntry {
    
    private DoubleArrayLogEntry baseLogger;

    public Pose2dLogEntry(DataLog log, String name) {
        this.baseLogger = new DoubleArrayLogEntry(log, name);
    }

    public void appendRadians(Pose2d pose){
        double[] data = new double[3];
        data[0] = pose.getTranslation().getX();
        data[1] = pose.getTranslation().getY();
        data[2] = pose.getRotation().getRadians();
        this.baseLogger.append(data);
    }

    public void appendDegrees(Pose2d pose){
        double[] data = new double[3];
        data[0] = pose.getTranslation().getX();
        data[1] = pose.getTranslation().getY();
        data[2] = pose.getRotation().getDegrees();
        this.baseLogger.append(data);
    }
}
