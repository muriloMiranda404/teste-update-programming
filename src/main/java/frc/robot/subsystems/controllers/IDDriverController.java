package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDDriverController {
    
    Trigger a();

    Trigger b();
    
    Trigger x();
    
    Trigger y();

    Trigger rightBumper();

    Trigger leftBumper();
}
