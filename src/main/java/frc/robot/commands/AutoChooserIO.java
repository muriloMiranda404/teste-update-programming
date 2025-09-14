package frc.robot.commands;

public interface AutoChooserIO {
    String getPathName();

    void addAutoNameCommand(String name, String pathName);
}
