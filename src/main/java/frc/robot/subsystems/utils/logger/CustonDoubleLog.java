package frc.robot.subsystems.utils.logger;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustonDoubleLog extends DoubleLogEntry{
    
    private static boolean isFms;
    private String name;
    private double loggedValue;

    public CustonDoubleLog(String name){
        super(DataLogManager.getLog(), name);
        this.name = name;
        CustonDoubleLog.isFms = DriverStation.getMatchNumber() > 0;
        this.loggedValue = 0;
    }

    @Override
    public void append(double value) {
        if(DriverStation.isEnabled() && value != loggedValue){
            this.loggedValue = value;
            super.append(value);
        }
        if(!isFms){
            SmartDashboard.putNumber(name, value);
        }
    }
}
