package frc.robot.subsystems.utils.logger;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustonStringLog extends StringLogEntry{
    
    private static boolean isFms;
    private String loggedValue;
    private String name;

    public CustonStringLog(String name){
        super(DataLogManager.getLog(), name);
        this.name = name;
        CustonStringLog.isFms = DriverStation.getMatchNumber() > 0;
        this.loggedValue = "";
        this.append("");
    }

    @Override
    public void append(String value){
        if(DriverStation.isEnabled() && loggedValue != value){
            loggedValue = value;
            super.append(value);
        }
        if(!CustonStringLog.isFms){
            SmartDashboard.putString(this.name, value);
        }
    }
}
