package frc.robot.commands.autonomousChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser implements AutoChooserIO{

    SendableChooser<String> autoChooser;
    public static AutoChooser mInstance = null;

    private AutoChooser(){
        this.autoChooser = new SendableChooser<>();

        this.autoChooser.setDefaultOption("no auto for use", "no auto for use");
        this.autoChooser.addOption("center autonomous", "New Auto");
    }

    public static AutoChooser getInstance(){
        if(mInstance == null){
            mInstance = new AutoChooser();
        }
        return mInstance;
    }

    @Override
    public String getPathName() {
        return this.autoChooser.getSelected();
    }

    @Override
    public void addAutoNameCommand(String name, String pathName) {
        this.autoChooser.addOption(name, pathName);
    }
}
