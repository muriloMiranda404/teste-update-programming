package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDMechanism {
    
    Trigger getCoralButton();

    Trigger throwCoralOnIntake();

    Trigger L1Button();

    Trigger L2Button();

    Trigger L3Button();

    Trigger L4Button();

    Trigger ProcessorButton();

    Trigger L2Algae();

    Trigger L3Algae();

    double UpElevator();

    double getSetpoint();

    XboxController getHID();

    boolean joystickIsNothingUsingMechanism();
}
