package frc.FRC9485.utils.JoystickChooser;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC9485.Autonomous.autonomousChooser.ChooserIO;

public class JoystickChooser extends SubsystemBase implements ChooserIO{
    
    LoggedDashboardChooser<String> joystickChooser;

    public static JoystickChooser mInstance = null;

    private JoystickChooser(){

        joystickChooser = new LoggedDashboardChooser<>("joystick chooser");

        joystickChooser.addOption("keyboard", "keyboard");
        joystickChooser.addOption("joystick", "joystick");
    }

    @Override
    public void periodic() {
        System.out.println("escolhido: " + joystickChooser.get());
    }

    public static JoystickChooser getInstance(){
        if(mInstance == null){
            mInstance = new JoystickChooser();
        }
        return mInstance;
    }

    @Override
    public String getChoosed(){
        return joystickChooser.get();
    }
    
    public SendableChooser<String> getChooser(){
        return joystickChooser.getSendableChooser();
    }

    @Override
    public void addCommand(String name, String value) {
        joystickChooser.addOption(name, value);
    }
}
