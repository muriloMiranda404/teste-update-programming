package frc.robot.commands.autonomousChooser;

public interface AutoChooserIO {
    String getPathName();

    void addAutoNameCommand(String name, String pathName);
}
