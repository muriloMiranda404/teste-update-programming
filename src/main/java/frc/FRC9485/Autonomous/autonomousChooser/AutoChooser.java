package frc.FRC9485.Autonomous.autonomousChooser;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser implements ChooserIO{

    LoggedDashboardChooser<String> autoChooser;
    public static AutoChooser mInstance = null;

    private AutoChooser(){
        this.autoChooser = new LoggedDashboardChooser<>("sendable chooser");

        this.autoChooser.addDefaultOption("is not used", "null");
        this.autoChooser.addOption("center autonomous", "New Auto");
    }

    public static AutoChooser getInstance(){
        if(mInstance == null){
            mInstance = new AutoChooser();
        }
        return mInstance;
    }

    @Override
    public String getChoosed() {
        return this.autoChooser.get();
    }

    @Override
    public void addCommand(String name, String value) {
        this.autoChooser.addOption(name, value);
    }
}
