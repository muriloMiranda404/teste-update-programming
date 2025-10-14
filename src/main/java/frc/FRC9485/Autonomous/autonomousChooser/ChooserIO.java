package frc.FRC9485.Autonomous.autonomousChooser;

public interface ChooserIO {

    String getChoosed();

    void addCommand(String name, String value);
}
