package frc.robot.subsystems.utils.logger;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustonBooleanLog extends BooleanLogEntry{

    private static boolean isFms;
    private String name;
    private Boolean loggedValue;
    
    public CustonBooleanLog(String name){
        super(DataLogManager.getLog(), name);
        this.name = name;
        CustonBooleanLog.isFms = DriverStation.getMatchNumber() > 0;
        this.loggedValue = true;
        append(false);
    }

    @Override
    public void append(boolean value) {
        if(DriverStation.isEnabled() && value != loggedValue){
            loggedValue = value;
            super.append(value);
        }
        if(!isFms){
            SmartDashboard.putBoolean(name, value);
        }
    }
}
