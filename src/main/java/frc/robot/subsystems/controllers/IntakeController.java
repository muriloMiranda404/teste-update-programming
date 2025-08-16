package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeController extends CommandXboxController{
    
    public static int id;
    public static IntakeController controller = new IntakeController(1);

    public IntakeController(int id){
        super(id);
    }

    public Trigger getCoralButton(){
        return controller.rightBumper();
    }

    public Trigger throwCoralOnIntake(){
        return controller.leftBumper();
    }

    public Trigger AlingRobotOnReef(){
        return controller.pov(270);
    }

    public Trigger L1Button(){
        return controller.a();
    }

    public Trigger L2Button(){
        return controller.b();
    }

    public Trigger L3Button(){
        return controller.y();
    }

    public Trigger L4Button(){
        return controller.x();
    }
    
}
